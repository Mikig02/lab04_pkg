import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import tf_transformations
import math
import numpy as np

class Dwa_node(Node):
    def __init__(self):
        super().__init__('dwa_node')
        self.get_logger().info('dwa_node started.')
        # === PARAMETRI ===
        self.declare_parameter("dt", 0.1) # time interval for simulation
        self.declare_parameter("sim_time", 2.0) #simulation time for each trajectory
        self.declare_parameter("time_granularity", 0.1) #time granularity for simulation

        self.declare_parameter("v_samples", 10) #how many values of linear velocity to sample
        self.declare_parameter("w_samples", 20) #how many values of angular velocity to sample

        self.declare_parameter("goal_dist_tol", 0.3) # distance from goal to consider it reached
        self.declare_parameter("collision_tol", 0.3) # distance from obstacles to consider a collision

        self.declare_parameter("weight_angle", 0.04) # weight for angle in objective function
        self.declare_parameter("weight_vel", 0.2) # weight for velocity in objective function
        self.declare_parameter("weight_obs", 0.1) # weight for obstacles in objective function
        self.declare_parameter("obstacle_max_dist", 3.0) # maximum distance to consider obstacles
        self.declare_parameter("max_num_steps", 300) # maximum number of steps to reach the goal
        self.declare_parameter("obst_tolerance", 0.5) # obstacle tolerance distance
        self.declare_parameter("frequency", 15.0) # go_to_pose_callback frequency
        self.declare_parameter("num_ranges", 27) # ranges to consider from laser scan
        


        self.dt = self.get_parameter("dt").value
        self.sim_time = self.get_parameter("sim_time").value
        self.time_granularity = self.get_parameter("time_granularity").value

        self.v_samples = self.get_parameter("v_samples").value
        self.w_samples = self.get_parameter("w_samples").value

        self.goal_dist_tol = self.get_parameter("goal_dist_tol").value
        self.collision_tol = self.get_parameter("collision_tol").value

        self.weight_angle = self.get_parameter("weight_angle").value
        self.weight_vel = self.get_parameter("weight_vel").value
        self.weight_obs = self.get_parameter("weight_obs").value

        self.obstacle_max_dist = self.get_parameter("obstacle_max_dist").value
        self.max_num_steps = self.get_parameter("max_num_steps").value
        self.obst_tolerance = self.get_parameter("obst_tolerance").value 
        self.frequency = self.get_parameter("frequency").value
        self.num_ranges = self.get_parameter("num_ranges").value
        self.goal_received = False
        self.goal_x = None
        self.goal_y = None

        

        
        #=== TIMER ===
        self.timer = self.create_timer(1.0 / self.frequency, self.go_to_pose_callback)

       # === PUB / SUB ===
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.goal_sub = self.create_subscription(Odometry,'/dynamic_goal_pose',self.goal_callback,10)

    def odom_callback(self, msg: Odometry):
        self.x = msg.pose.pose.position.x # get x position
        self.y = msg.pose.pose.position.y # get y position
        q = msg.pose.pose.orientation # get orientation quaternion
        quat = [q.x, q.y, q.z, q.w] # convert quaternion to list
        _, _, self.yaw = tf_transformations.euler_from_quaternion(quat) # get yaw from quaternion

    def scan_callback(self, msg: LaserScan): #callback for laser scan, chiedi a Michele parere su ciclo for
     self.obstacles1 = np.array(msg.ranges)
     self.obstacles1 = np.nan_to_num(  # convert NaN and inf to numbers
     self.obstacles1,
     nan=msg.range_min,
     posinf=msg.range_max
        )
     self.obstacles = np.zeros(len(self.obstacles1))
     for i in range(len(self.obstacles1)): 
         if self.obstacles1[i] <=3.5 :
             self.obstacles[i] = self.obstacles1[i]
         else:
            self.obstacles[i] = np.inf
        #  if self.obstacles[i]  <= self.obst_tolerance:
        #     self.get_logger().info('detected obstacle at: {:.2f} m'.format(self.obstacles[i])) 
    
     #Filter scan ranges 
     self.filtered_obstacles = np.zeros(self.num_ranges) #array to store filtered obstacles of dimension num_ranges(ossia quanti settori voglio dividere il campo visivo del laser)
     step = int(len(self.obstacles)/self.num_ranges) #dimensione vari settori
     for i in range(self.num_ranges):
        self.filtered_obstacles[i] = min(self.obstacles[i*step:(i+1)*step]) # prendo il minimo di ogni settore e lo metto in filtered_obstacles
    

     # ultima parte da vedere insieme a Michele
     self.obstacles_xy = []    # <--- lista delle coordinate (x,y) degli ostacoli

     angle_min = msg.angle_min #prende l'angolo minimo del laser
     angle_inc = msg.angle_increment #passo angolare del laser

     # angolo centrale di ogni settore
     for i in range(int(self.num_ranges)):
        if self.filtered_obstacles[i] == np.inf:
            continue  # salta se non c'è ostacolo in questo settore
        dist = self.filtered_obstacles[i]

        # calcolo dell’indice del centro del settore
        center_idx = int(i*step + step/2)

        # angolo relativo del laser
        angle = angle_min + center_idx * angle_inc

        # coordinate ostacolo nel frame del ROBOT
        ox_r = dist * np.cos(angle)
        oy_r = dist * np.sin(angle)

        # trasformazione nel WORLD frame
        ox_w = self.x + ox_r * np.cos(self.yaw) - oy_r * np.sin(self.yaw)
        oy_w = self.y + ox_r * np.sin(self.yaw) + oy_r * np.cos(self.yaw)

        self.obstacles_xy.append([ox_w, oy_w])

     # convertiamo in numpy array
     self.obstacles_xy = np.array(self.obstacles_xy)

     # ora self.obstacles_xy è ciò che DEVI dare all’algoritmo DWA

    def goal_callback(self, msg: Odometry): #goal callback to get dynamic goal (GoalManager node)
        self.goal_x = msg.pose.pose.position.x
        self.goal_y = msg.pose.pose.position.y
        self.goal_received = True
        self.get_logger().info('New goal received: ({:.2f}, {:.2f})'.format(self.goal_x, self.goal_y))


    def go_to_pose_callback(self): #core of the program, this generates command to reach a goal
       
     if not self.goal_received:
          self.get_logger().info("Waiting for goal...")
          return
     
       #safety check
     if np.min(self.filtered_obstacles) < 0.25:
              self.get_logger().info('Obstacle too close! Stopping robot.')
              cmd = Twist()
              cmd.linear.x = 0.0
              cmd.angular.z = 0.0
              self.cmd_pub.publish(cmd)
              return
       
            
     
     

      
            
    



    