import time
import threading
import psutil
import csv
import os
from datetime import datetime
from pynvml import (
    nvmlInit, nvmlShutdown,
    nvmlDeviceGetHandleByIndex,
    nvmlDeviceGetUtilizationRates,
    nvmlDeviceGetMemoryInfo,
    nvmlDeviceGetTemperature,
    NVML_TEMPERATURE_GPU
)
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rospy

class PerformanceLogger:
    def __init__(self, log_file='performance_log.csv', interval=0.01):
        self.interval = interval  # Time between samples (seconds)
        self.running = False
        self.thread = None
        self.data = []
        self.cmd_vel_lin = 0.0
        self.cmd_vel_ang = 0.0
        self.pose = [0.0, 0.0]
        self.ori = [0.0, 0.0, 0.0, 0.0]

        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")

        log_dir = '/home/interns/CrowdSurfer_ws/logs/FASTLIO_tests_CSV/DC'
        os.makedirs(log_dir, exist_ok=True)
        self.log_file = f"{log_dir}/log_{timestamp}.csv"

        # Create CSV file with headers if not already present
        if not os.path.exists(self.log_file):
            with open(self.log_file, mode='w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['timestamp', 'cpu_percent', 'ram_percent', 'gpu_percent', 'vram_percent', 'cpuT', 'gpuT', 'linVel', 'angVel', 'posX', 'posY', 'oriW', 'oriX', 'oriY', 'oriZ'])

    def vel_callback(self, msg):
        linear = msg.linear
        angular = msg.angular

        self.cmd_vel_lin = linear.x
        self.cmd_vel_ang = angular.z

    def pos_callback(self, msg):
        pos = msg.pose.pose.position
        orin = msg.pose.pose.orientation

        self.pose = [pos.x, pos.y]
        self.ori = [orin.w, orin.x, orin.y, orin.z]

    def _log_metrics(self):
        nvmlInit()
        handle = nvmlDeviceGetHandleByIndex(0)  # Assumes only 1 GPU

        while self.running:
            timestamp = datetime.now().isoformat()
            rospy.Subscriber('wheelchair_diff/cmd_vel', Twist, self.vel_callback)
            rospy.Subscriber('/Odometry', Odometry, self.pos_callback)
            cpu = psutil.cpu_percent(interval=None)
            ram = psutil.virtual_memory().percent

            temps = psutil.sensors_temperatures()
            cpu_temp = temps['k10temp'][0].current

            gpu_temp = nvmlDeviceGetTemperature(handle, NVML_TEMPERATURE_GPU)

            gpu_util = nvmlDeviceGetUtilizationRates(handle)
            gpu = gpu_util.gpu
            vram_info = nvmlDeviceGetMemoryInfo(handle)
            vram = (vram_info.used / vram_info.total) * 100

            # Append to CSV file
            if self.cmd_vel_lin != 0.0:
                with open(self.log_file, mode='a', newline='') as f:
                    writer = csv.writer(f)
                    writer.writerow([timestamp, cpu, ram, gpu, vram, cpu_temp, gpu_temp, self.cmd_vel_lin, self.cmd_vel_ang, self.pose[0], self.pose[1], self.ori[0], self.ori[1], self.ori[2], self.ori[3]])

            time.sleep(self.interval)

        nvmlShutdown()

    def start_logging(self):
        self.running = True
        self.data = []
        self.thread = threading.Thread(target=self._log_metrics)
        self.thread.start()

    def stop_logging(self):
        self.running = False
        if self.thread:
            self.thread.join()
        return self._compute_statistics()

    def _compute_statistics(self):
        if not self.data:
            return {}

        num_samples = len(self.data)
        avg = {
            'cpu_avg': sum(d['cpu'] for d in self.data) / num_samples,
            'ram_avg': sum(d['ram'] for d in self.data) / num_samples,
            'gpu_avg': sum(d['gpu'] for d in self.data) / num_samples,
            'vram_avg': sum(d['vram'] for d in self.data) / num_samples,
        }

        max_vals = {
            'cpu_max': max(d['cpu'] for d in self.data),
            'ram_max': max(d['ram'] for d in self.data),
            'gpu_max': max(d['gpu'] for d in self.data),
            'vram_max': max(d['vram'] for d in self.data),
        }

        return {**avg, **max_vals}