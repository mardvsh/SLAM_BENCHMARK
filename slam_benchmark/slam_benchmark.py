import os
import subprocess
import time
import threading
import psutil
import matplotlib.pyplot as plt
import cv2
import numpy as np
from skimage.metrics import structural_similarity as ssim

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

def run_process(cmd):
    return subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

def monitor_resources(name, duration=180):
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

def compare_maps(gen_map):
    ref = cv2.imread(REFERENCE_MAP, cv2.IMREAD_GRAYSCALE)
    gen = cv2.imread(gen_map, cv2.IMREAD_GRAYSCALE)
    if ref.shape != gen.shape:
        gen = cv2.resize(gen, (ref.shape[1], ref.shape[0]))
    mse_val = np.mean((ref - gen) ** 2)
    ssim_val = ssim(ref, gen)
    return mse_val, ssim_val

def plot_metrics(name, cpu, ram):
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

def kill_previous_processes():
    """ Завершаем все процессы Gazebo, ROS и rviz """
    subprocess.run("pkill -f gzserver", shell=True)
    subprocess.run("pkill -f gzclient", shell=True)
    subprocess.run("pkill -f rviz", shell=True)
    time.sleep(5)  # Даём время на завершение процессов

def reset_simulation():
    """ Сбрасываем робота на базовую позицию перед запуском нового теста """
    subprocess.run("rosservice call /gazebo/reset_world", shell=True)
    subprocess.run("rosservice call /gazebo/reset_simulation", shell=True)
    time.sleep(5)  # Даём время на сброс симуляции

def launch_all():
    results = []
    for name, launch_cmd in ALGORITHMS.items():
        print(f"\n>>> Starting {name.upper()}...")

        # Завершаем предыдущие процессы
        kill_previous_processes()

        # Запускаем симуляцию
        gazebo = run_process("roslaunch turtlebot3_gazebo turtlebot3_world.launch")
        time.sleep(10)  # Подождать, пока запустится Gazebo

        # Теперь можно безопасно сбросить симуляцию
        reset_simulation()

        # Запускаем выбранный SLAM
        roslaunch = run_process(f"roslaunch {launch_cmd}")
        time.sleep(10)

        # Автопроходка
        driver = run_process("python3 auto_drive.py")

        # Сбор метрик
        cpu_thread = threading.Thread(target=monitor_resources, args=(name,))
        cpu_thread.start()

        driver.wait()
        cpu_thread.join()
        
        print(f"[+] Saving map for {name}")
        subprocess.run(f"rosrun map_server map_saver -f ./maps/{name}", shell=True)

        roslaunch.terminate()
        gazebo.terminate()
        time.sleep(5)

        gen_map = os.path.join(MAPS_DIR, f"{name}.pgm")
        mse, ssim_val = compare_maps(gen_map)
        cpu_log = np.loadtxt(f"{LOGS_DIR}/{name}_cpu.txt")
        ram_log = np.loadtxt(f"{LOGS_DIR}/{name}_ram.txt")

        plot_metrics(name, cpu_log, ram_log)
        
        result = {
            'name': name,
            'MSE': round(mse, 2),
            'SSIM': round(ssim_val, 3),
            'CPU_avg': round(np.mean(cpu_log), 1),
            'RAM_avg': round(np.mean(ram_log), 1)
        }
        results.append(result)

    return results

def print_results(results):
    print("\n================ RESULTS ================")
    print(f"{'Algorithm':<15} {'MSE':<10} {'SSIM':<10} {'CPU (%)':<10} {'RAM (MB)':<10}")
    for r in results:
        print(f"{r['name']:<15} {r['MSE']:<10} {r['SSIM']:<10} {r['CPU_avg']:<10} {r['RAM_avg']:<10}")

if __name__ == '__main__':
    all_results = launch_all()
    print_results(all_results)
