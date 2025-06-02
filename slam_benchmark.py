import os
import subprocess
import time
import threading
import psutil
import matplotlib.pyplot as plt
import cv2
import numpy as np
from skimage.metrics import peak_signal_noise_ratio as psnr

# Настройки
ALGORITHMS = {
    'gmapping': 'turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping',
    'cartographer': 'turtlebot3_slam turtlebot3_slam.launch slam_methods:=cartographer',
    'hector': 'turtlebot3_slam turtlebot3_slam.launch slam_methods:=hector',
    'karto': 'turtlebot3_slam turtlebot3_slam.launch slam_methods:=karto',
    'frontier': 'turtlebot3_slam turtlebot3_slam.launch slam_methods:=frontier_exploration'
}

REFERENCE_MAP = 'reference_map.pgm'
MAPS_DIR = 'maps'
LOGS_DIR = 'logs'
os.makedirs(MAPS_DIR, exist_ok=True)
os.makedirs(LOGS_DIR, exist_ok=True)

# Проверка наличия ROS
def check_ros_setup():
    if subprocess.run("which rosrun", shell=True, stdout=subprocess.DEVNULL).returncode != 0:
        raise EnvironmentError("ROS не настроен. Проверьте, что вы запустили 'source /opt/ros/<distro>/setup.bash' и 'source ~/catkin_ws/devel/setup.bash'")

# Запуск команды
def run_process(cmd):
    return subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

# Мониторинг ресурсов
def monitor_resources(name, duration=20):
    cpu_log = []
    ram_log = []
    for _ in range(duration):
        cpu = psutil.cpu_percent(interval=1)
        ram = psutil.virtual_memory().used / (1024 * 1024)
        cpu_log.append(cpu)
        ram_log.append(ram)
    np.savetxt(f"{LOGS_DIR}/{name}_cpu.txt", cpu_log)
    np.savetxt(f"{LOGS_DIR}/{name}_ram.txt", ram_log)
    return cpu_log, ram_log

# Сравнение карт
def compare_maps(gen_map):
    if not os.path.exists(REFERENCE_MAP):
        raise FileNotFoundError(f"Файл {REFERENCE_MAP} не найден")
    if not os.path.exists(gen_map):
        raise FileNotFoundError(f"Сгенерированная карта {gen_map} не найдена")
    ref = cv2.imread(REFERENCE_MAP, cv2.IMREAD_GRAYSCALE)
    gen = cv2.imread(gen_map, cv2.IMREAD_GRAYSCALE)
    if ref is None or gen is None:
        raise ValueError("Ошибка при чтении одной из карт (NoneType)")
    if ref.shape != gen.shape:
        gen = cv2.resize(gen, (ref.shape[1], ref.shape[0]))
    mse_val = np.mean((ref - gen) ** 2)
    psnr_val = psnr(ref, gen)
    return mse_val, psnr_val

# Графики
def plot_metrics(name, cpu, ram, mse, psnr_val):
    plt.figure()
    plt.plot(cpu, label='CPU %')
    plt.plot(ram, label='RAM MB')
    plt.title(f"Resources for {name}")
    plt.xlabel("Time (s)")
    plt.ylabel("Usage")
    plt.legend()
    plt.grid()
    plt.savefig(f"{LOGS_DIR}/{name}_resources.png")
    plt.close()

    plt.figure()
    plt.bar(['MSE', 'PSNR'], [mse, psnr_val])
    plt.title(f"MSE and PSNR for {name}")
    plt.ylabel("Value")
    plt.savefig(f"{LOGS_DIR}/{name}_mse_psnr.png")
    plt.close()

# Завершение старых процессов
def kill_previous_processes():
    subprocess.run("pkill -f gzserver", shell=True)
    subprocess.run("pkill -f gzclient", shell=True)
    subprocess.run("pkill -f rviz", shell=True)
    time.sleep(5)

# Сброс симуляции
def reset_simulation():
    subprocess.run("rosservice call /gazebo/reset_world", shell=True)
    subprocess.run("rosservice call /gazebo/reset_simulation", shell=True)
    time.sleep(5)

# Основной запуск
def launch_all():
    check_ros_setup()
    results = []
    for name, launch_cmd in ALGORITHMS.items():
        print(f"\n>>> Starting {name.upper()}...")

        kill_previous_processes()
        gazebo = run_process("roslaunch turtlebot3_gazebo turtlebot3_world.launch")
        time.sleep(20)
        reset_simulation()

        roslaunch = run_process(f"roslaunch {launch_cmd}")
        time.sleep(20)

        start_time = time.time()
        driver = run_process("python3 auto_drive.py")

        cpu_thread = threading.Thread(target=monitor_resources, args=(name, 20))
        cpu_thread.start()

        driver.wait()
        end_time = time.time()
        elapsed_time = round(end_time - start_time, 1)

        cpu_thread.join()

        print(f"[+] Saving map for {name}")
        map_save_result = subprocess.run(f"rosrun map_server map_saver -f ./maps/{name}", shell=True)
        if map_save_result.returncode != 0:
            print(f"[!] Ошибка при сохранении карты для {name}")
            continue

        roslaunch.terminate()
        gazebo.terminate()
        time.sleep(5)

        gen_map = os.path.join(MAPS_DIR, f"{name}.pgm")

        try:
            mse, psnr_val = compare_maps(gen_map)
        except Exception as e:
            print(f"[!] Ошибка при сравнении карт: {e}")
            continue

        cpu_log = np.loadtxt(f"{LOGS_DIR}/{name}_cpu.txt")
        ram_log = np.loadtxt(f"{LOGS_DIR}/{name}_ram.txt")

        plot_metrics(name, cpu_log, ram_log, mse, psnr_val)

        result = {
            'name': name,
            'MSE': round(mse, 2),
            'PSNR': round(psnr_val, 2),
            'CPU_avg': round(np.mean(cpu_log), 1),
            'RAM_avg': round(np.mean(ram_log), 1),
            'Time_s': elapsed_time
        }
        results.append(result)

    return results

# Печать результатов
def print_results(results):
    print("\n================ RESULTS ================")
    print(f"{'Algorithm':<15} {'MSE':<10} {'PSNR':<10} {'CPU (%)':<10} {'RAM (MB)':<10}")
    for r in results:
        print(f"{r['name']:<15} {r['MSE']:<10} {r['PSNR']:<10} {r['CPU_avg']:<10} {r['RAM_avg']:<10}")

if __name__ == '__main__':
    all_results = launch_all()
    print_results(all_results)
