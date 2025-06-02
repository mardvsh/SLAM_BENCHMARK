import os
import subprocess
import time
import threading
import psutil
import matplotlib.pyplot as plt
import cv2
import numpy as np
from skimage.metrics import peak_signal_noise_ratio as psnr
from methods import method_a, method_b, method_c, method_d, method_e
from itertools import combinations


MAPS_DIR = 'maps'
LOGS_DIR = 'logs'
REFERENCE_MAP = 'reference_map.pgm'
os.makedirs(MAPS_DIR, exist_ok=True)
os.makedirs(LOGS_DIR, exist_ok=True)


METHOD_CONFIGS = []
methods_all = ['a', 'b', 'c', 'd', 'e']
for i in range(0, len(methods_all)+1):
    for combo in combinations(methods_all, i):
        METHOD_CONFIGS.append(list(combo))


def run_process(cmd):
    return subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

def monitor_resources(name, duration=20):
    cpu_log, ram_log = [], []
    for _ in range(duration):
        cpu_log.append(psutil.cpu_percent(interval=1))
        ram_log.append(psutil.virtual_memory().used / (1024 * 1024))
    np.savetxt(f"{LOGS_DIR}/{name}_cpu.txt", cpu_log)
    np.savetxt(f"{LOGS_DIR}/{name}_ram.txt", ram_log)
    return cpu_log, ram_log

def compare_maps(gen_map):
    ref = cv2.imread(REFERENCE_MAP, cv2.IMREAD_GRAYSCALE)
    gen = cv2.imread(gen_map, cv2.IMREAD_GRAYSCALE)
    if ref.shape != gen.shape:
        gen = cv2.resize(gen, (ref.shape[1], ref.shape[0]))
    mse_val = np.mean((ref - gen) ** 2)
    psnr_val = psnr(ref, gen)
    return mse_val, psnr_val

def plot_metrics(name, cpu, ram, mse, psnr):
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
    plt.bar(['MSE', 'PSNR'], [mse, psnr])
    plt.title(f"MSE and PSNR for {name}")
    plt.ylabel("Value")
    plt.savefig(f"{LOGS_DIR}/{name}_mse_psnr.png")
    plt.close()

def kill_previous_processes():
    subprocess.run("pkill -f gzserver", shell=True)
    subprocess.run("pkill -f gzclient", shell=True)
    subprocess.run("pkill -f rviz", shell=True)
    time.sleep(5)

def reset_simulation():
    subprocess.run("rosservice call /gazebo/reset_world", shell=True)
    subprocess.run("rosservice call /gazebo/reset_simulation", shell=True)
    time.sleep(5)

def apply_methods(data, methods):
    if 'a' in methods:
        scan_a = method_a.ScanWithOdometry(data['odom'], data['lidar'])
        data['lidar'] = scan_a.apply_odometry()

    if 'b' in methods:
        scan_b = method_b.WeightedFilter(data['lidar'])
        data['lidar'] = scan_b.apply_filter()[:, :2]

    if 'c' in methods:
        scan_c = method_c.SpatialTemporalFilter(data['lidar'], data['odom'])
        data['lidar'] = scan_c.apply_filter()

    if 'd' in methods:
        scan_d = method_d.OdometrySmoother(data['lidar'])
        data['lidar'] = scan_d.apply_noise()

    if 'e' in methods:
        scan_f = method_e.TemporalSmoother(data['lidar'])
        data['lidar'] = scan_f.apply_smoothing()

    return data

def run_config(methods):
    name = "gmapping_" + "".join(methods) if methods else "gmapping_base"
    print(f"\n>>> Starting {name.upper()}...")

    kill_previous_processes()
    gazebo = run_process("roslaunch turtlebot3_gazebo turtlebot3_world.launch")
    time.sleep(10)
    reset_simulation()

    roslaunch = run_process("roslaunch turtlebot3_slam turtlebot3_slam.launch slam_methods:=gmapping")
    time.sleep(10)

    # Имитация получения сенсорных данных
    dummy_data = {
        'odom': {'x': 0.5, 'y': 0.2, 'theta': np.pi/6},
        'lidar': np.random.rand(100, 2)
    }
    filtered_data = apply_methods(dummy_data, methods)

    start_time = time.time()
    driver = run_process("python3 auto_drive.py")

    cpu_thread = threading.Thread(target=monitor_resources, args=(name, 20))
    cpu_thread.start()

    driver.wait()
    end_time = time.time()
    elapsed_time = round(end_time - start_time, 1)

    cpu_thread.join()

    print(f"[+] Saving map for {name}")
    subprocess.run(f"rosrun map_server map_saver -f ./maps/{name}", shell=True)

    roslaunch.terminate()
    gazebo.terminate()
    time.sleep(5)

    gen_map = os.path.join(MAPS_DIR, f"{name}.pgm")
    mse, psnr_val = compare_maps(gen_map)
    cpu_log = np.loadtxt(f"{LOGS_DIR}/{name}_cpu.txt")
    ram_log = np.loadtxt(f"{LOGS_DIR}/{name}_ram.txt")

    plot_metrics(name, cpu_log, ram_log, mse, psnr_val)

    return {
        'name': name,
        'MSE': round(mse, 2),
        'PSNR': round(psnr_val, 2),
        'CPU_avg': round(np.mean(cpu_log), 1),
        'RAM_avg': round(np.mean(ram_log), 1),
        'Time_s': elapsed_time
    }

def print_results(results):
    print("\n================ RESULTS ================")
    print(f"{'Config':<15} {'MSE':<10} {'PSNR':<10} {'CPU (%)':<10} {'RAM (MB)':<10}")
    for r in results:
        print(f"{r['name']:<15} {r['MSE']:<10} {r['PSNR']:<10} {r['CPU_avg']:<10} {r['RAM_avg']:<10}")

if __name__ == '__main__':
    all_results = []
    for methods in METHOD_CONFIGS:
        result = run_config(methods)
        all_results.append(result)
    print_results(all_results)
