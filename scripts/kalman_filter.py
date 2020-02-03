#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist, PoseStamped
from sensor_msgs.msg import LaserScan
import numpy as np
import tf
import matplotlib.pyplot as plt

dt = 0.05
baselink_offset = 0.08 # distance between front end of the robot and base_link
camera_offset = 0.03 # distance between camera and base_link

class KalmanFilter:
    def __init__(self, task, F = None, H = None, B = None, P = None, Q = None, R = None):
        rospy.init_node('kalman_filter')
        self.scansub = rospy.Subscriber('scan', LaserScan, self.scan_callback)
        self.scansub = rospy.Subscriber('camera_scan', LaserScan, self.camerascan_callback)
        self.velsub = rospy.Subscriber('cmd_vel', Twist, self.vel_callback)
        self.posesub = rospy.Subscriber('pose', PoseStamped, self.pose_callback)
        self.initposesub = rospy.Subscriber('initial_pose', PoseWithCovarianceStamped, self.initpose_callback)
        self.posepub = rospy.Publisher('pose_kf', PoseWithCovarianceStamped, queue_size = 0)
        self.pose_kf = PoseWithCovarianceStamped()

        self.task = task
        self.is_moving = False

        self.n = F.shape[0] # dimension of state
        self.k = H.shape[0] # dimension of observation
        self.l = B.shape[0] # dimension of control

        self.F = F # state transition model n*n
        self.H = H # observation model k*n
        self.B = B # control-input model n*l
        self.Q = np.eye(self.n) if Q is None else Q # covariance of process noise
        self.R = np.eye(self.n) if R is None else R # covariance of observation noise
        self.P = np.eye(self.n) if P is None else P # posteriori estimate covariance matrix 
        self.x = np.zeros((self.n, 1)) # state x_t/t
        self.x_predict = np.zeros((self.n, 1)) # predict state x_t+1/t
        self.get_initial_pose = False
        self.get_initial_pose_from_pose = False
        self.initial_pose_x = 0
        self.last_pose_x = 0
        self.distance = 2
        self.last_distance = 2

        self.z = np.zeros((self.k, 1)) # sensor reading
        self.u = np.zeros((self.l, 1)) # control

        self.rate = rospy.Rate(1/dt)
    
    def scan_callback(self, msg):
        if self.task == 3 or self.task == 4: return
        if msg.ranges[0] < 10:
            self.z = 2 - (msg.ranges[0]-baselink_offset) # front distance
    
    def camerascan_callback(self, msg):
        if self.task == 1 or self.task == 2: return
        length = len(msg.ranges)
        distance = self.last_distance
        n = 0

        for i in range(length/2 - 5, length/2 + 5):
            if msg.ranges[i] > 0.05 and msg.ranges[i] < self.last_distance:
                distance += msg.ranges[i]
                n = n + 1

        if n == 0: distance = self.last_distance 
        else: 
            distance = distance/n
            if distance < self.last_distance: self.last_distance = distance
            else: distance = self.last_distance
        print("distance:", distance)
            
        self.z = 2 - (distance-camera_offset-baselink_offset) # front distance

    def vel_callback(self, msg):
        if self.task == 1: self.u = msg.linear.x

        if msg.linear.x != 0: self.is_moving = True
        else: self.is_moving = False

    def initpose_callback(self, msg):
        if self.get_initial_pose: return
        self.get_initial_pose = True
        self.x = msg.pose.pose.position.x

    def pose_callback(self, msg):
        if not self.get_initial_pose_from_pose: 
            self.last_pose_x = msg.pose.position.x
            self.get_initial_pose_from_pose = True
            self.initial_pose_x = msg.pose.position.x

        if self.task == 2 or self.task == 4: self.u = msg.pose.position.x - self.last_pose_x
        self.last_pose_x = msg.pose.position.x

    def predict(self):
        self.x_predict = np.dot(self.F, self.x) + np.dot(self.B, self.u)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) +self.Q

    def update(self):
        r = self.z - np.dot(self.H, self.x) # difference between expected and "true"
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R # covariance of sensor reading
        K = np.dot(np.dot(self.P, self.H.T), (1/S)) # Kalman gain
        self.x = self.x_predict + np.dot(K, r)
        I = np.eye(self.n)
        self.P = np.dot((I - np.dot(K, self.H)), self.P)

        self.pose_kf.header.frame_id = "odom_kf"
        self.pose_kf.header.seq = 0
        self.pose_kf.header.stamp = rospy.Time.now()

        self.pose_kf.pose.pose.position.x = self.x
        self.pose_kf.pose.covariance[0] = self.P
        self.posepub.publish(self.pose_kf)

        br = tf.TransformBroadcaster()
        br.sendTransform((self.x, 0, 0), (0, 0, 0, 1), rospy.Time.now(), "base_footprint","odom_kf")


if __name__ == "__main__":
    print("1 for cmd_vel and scan")
    print("2 for pose and scan")
    print("3 for cmd_vel and scan from depth camera")
    print("4 for pose and scan from depth camera")

    tasks=['cmd_vel and scan', 
           'pose and scan',
           'cmd_vel and scan from depth camera',
           'pose and scan from depth camera']

    task = input("Choose Kalman Filter:\n")
    if task == 1:
        B = np.array([dt])
    elif task == 2:
        B = np.array([1])
    elif task == 3:
        B = np.array([dt])
    elif task == 4:
        B = np.array([1])
        
    print("using Kalman Filter with " + tasks[task-1] +"\n")
    F = np.array([1])
    H = np.array([1])
    Q = np.array([1])
    P = np.array([1000])
    R = np.array([1])

    kf = KalmanFilter(task, F, H, B, P, Q, R) # F,H,B,P,Q,R
    
    t = []
    current_time = 0
    t.append(current_time)
    x = []
    pose = []

    while not kf.get_initial_pose: continue
    print("get initial pose")
    x.append(kf.x)
    pose.append(kf.last_pose_x-kf.initial_pose_x)

    while not kf.is_moving: continue
    print("start moving")

    while kf.is_moving:
        kf.predict()
        kf.update()

        current_time += dt
        t.append(current_time)
        x.append(kf.x)
        pose.append(kf.last_pose_x-kf.initial_pose_x)
        print(current_time, kf.x)

        kf.rate.sleep()
    print("stop moving")

    plt.plot(t, x, label = 'x from kalman filter')
    plt.plot(t, [0]*len(t), 'r', linestyle='solid', label = 'ground truth')
    plt.plot(t, [1.1]*len(t), 'r', linestyle='solid')
    plt.title("Kalman Filter with " + tasks[task-1])
    plt.xlabel('time(s)')
    plt.ylabel('distance(m)')
    plt.legend(loc='upper left')
    plt.text(3, 1, "error: "+str(1.1 - kf.x))

    plt.show()