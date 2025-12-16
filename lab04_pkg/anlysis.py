import math
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rosbag2_reader_py import Rosbag2Reader   # oppure dal file dove hai la classe
from tf_transformations import euler_from_quaternion
from rclpy.time import Time
from scipy.interpolate import interp1d


bag_path = "/home/pier/ros2_ws/src/lab04_pkg/lab04_pkg/rosbag_lab04_task1_task2"
reader = Rosbag2Reader(bag_path)

status_topic = "/dwa/status"

topics_status = [status_topic]
reader.set_filter(topics_status)

# contatori
n_goal = 0
n_collision = 0
n_timeout = 0

for topic_name, msg, t in reader:
    if topic_name == status_topic and isinstance(msg, String):
        status = msg.data
        
        if status in ["Goal Reached"]:
            n_goal += 1
        elif status == "Collision":
            n_collision += 1
        elif status == "Timeout":
            n_timeout += 1
     
reader.reset_filter()

n_total = n_goal + n_collision + n_timeout
print(f"Total navigation events: {n_total}")
print(f"   Times the robot reach the goal : {n_goal}")
print(f"  Collision           : {n_collision}")
print(f"  Timeout             : {n_timeout}")

if n_total > 0:
    success_rate = 100.0 * n_goal / n_total
    print(f"\nSuccess Rate = {success_rate:.1f} %")
else:
    print("\nNo status messages found in the bag.")

follow_dist     = 0.2                 # distanza ideale che avevi nel DWA
dist_tol        = 0.3                 # tolleranza sulla distanza
bearing_tol_rad = math.radians(60.0)   # 60 gradi

robot_topic  = "/ground_truth"
target_topic = "/dynamic_goal_pose"

topics_tracking = [robot_topic, target_topic]
reader.set_filter(topics_tracking)

data = {
    robot_topic:  {"t": [], "x": [], "y": [], "theta": []},
    target_topic: {"t": [], "x": [], "y": [], "theta": []},
}

for topic_name, msg, t in reader:
    if topic_name in topics_tracking and isinstance(msg, Odometry):
        stamp = Time.from_msg(msg.header.stamp).nanoseconds * 1e-9

        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        _, _, theta = euler_from_quaternion([q.x, q.y, q.z, q.w])

        data[topic_name]["t"].append(stamp)
        data[topic_name]["x"].append(x)
        data[topic_name]["y"].append(y)
        data[topic_name]["theta"].append(theta)

reader.reset_filter()

# conversione in array numpy
robot_t  = np.array(data["/ground_truth"]["t"])
robot_x  = np.array(data["/ground_truth"]["x"])
robot_y  = np.array(data["/ground_truth"]["y"])
robot_th = np.array(data["/ground_truth"]["theta"])

target_t  = np.array(data["/dynamic_goal_pose"]["t"])
target_x  = np.array(data["/dynamic_goal_pose"]["x"])
target_y  = np.array(data["/dynamic_goal_pose"]["y"])
target_th = np.array(data["/dynamic_goal_pose"]["theta"])

print("Campioni robot:", len(robot_t))
print("Campioni target:", len(target_t))

if len(robot_t) < 2 or len(target_t) < 2:
    raise RuntimeError("Dati insufficienti per calcolare il time of tracking")
#interp1d richiede almeno 2 punti per costruire la funzione,con un solo punto non può fare interpolazione 

#================== INTERPOLAZIONE TARGET SUI TEMPI DEL ROBOT ==================

target_data = np.vstack((target_x, target_y, target_th)).T

interp_target = interp1d(
    target_t,
    target_data,
    axis=0,
    fill_value="extrapolate",
    kind="linear"
)

target_on_robot = interp_target(robot_t)
tgt_x = target_on_robot[:, 0]
tgt_y = target_on_robot[:, 1]
# tgt_th = target_on_robot[:, 2]  # se ti serve

# ================== DISTANZA E BEARING ==================

dx = tgt_x - robot_x
dy = tgt_y - robot_y
dist = np.hypot(dx, dy)

angle_to_target = np.arctan2(dy, dx)
bearing = angle_to_target - robot_th
bearing = (bearing + np.pi) % (2 * np.pi) - np.pi   # wrap in [-pi, pi]

dist_ok = np.abs(dist - follow_dist) <= dist_tol
bearing_ok = np.abs(bearing) <= bearing_tol_rad
tracking_mask = dist_ok & bearing_ok

# ================== INTEGRAZIONE NEL TEMPO ==================

dt = np.diff(robot_t, prepend=robot_t[0])  # dt[0] = 0
total_time = robot_t[-1] - robot_t[0]
tracking_time = np.sum(dt[tracking_mask])

tracking_percent = 100.0 * tracking_time / total_time

print("\n===== TIME OF TRACKING =====")
print(f"Total time:       {total_time:.2f} s")
print(f"Tracking time:    {tracking_time:.2f} s")
print(f"Time of tracking: {tracking_percent:.1f} %")