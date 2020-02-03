# kalman_filter

## How to use
(1)`source devel/setup.bash` and `catkin_make` as what we usually do in ros  
(2)`roslaunch kalman_filter kalmanfilter.launch`  
wait until it ask you to choose task, then wait until it said 'get initial pose'  
(3)`roslaunch kalman_filter kalmanfilter_with_depthimage.launch`  
bag file will be running. After several seconds, you can see the final prediction from kalman filter

## Implementation
### 1. Realize Kalman Filter  
Class Kalman_Filter initializes a node `kalman_filter`, subscribes to `cmd_vel`,`pose`,`initial_pose`,`scan`,`camera_scan` topics and can publish `pose_kf` topic. The main function are predict and update correspond to these two steps in Kalman Filter. Class input parameters inculdes F, B, H, P, Q, R which are parameter in Kalman Filter, and task which decide what control and sensor value the Kalman Filter will get to predict x. 

### 2. Different Tasks and Parameters
state x: distance from original pose x(2 - distance to the wall) 
state transition model F = 1  
observation model H = 1  
posteriori estimate covariance matrix P = 1000  
covariance of process noise Q = 1  
covariance of observation noise R = 1  
#### (1) using cmd_vel
control u: cmd_vel.linear.x  
Control-input model B: dt  
#### (2) using pose 
control u: pose.x - lastpose.x
Control-input model B: 1  
The fist lastpose is set to be the first pose, so the first u is 0.  
#### (3) using scan
observation z: scan.ranges[0] - distance from baselink to the front end of the robot
If this data is nan, I will use last data.   
#### (4) using camera_scan from depth camera  
observation z: average(camera_scan.ranges[middle]) - distance from camera to the front end of the robot
Camera_scan is published using depthimage_to_laserscan topic package. It is computed from depth image which is uncompressed from camera/depth/image_raw/compressed topic message in bag file.

### 3. Time step  
Time step is set globally with dt. The Kalman Filter is set to predict and update, then sleep for dt using rate.sleep(). Rate is 1/dt. If we want to get a better result, more work on getting a precise timing is necessary.

### 4. Publish and Plot Result
Predict result from Kalman Filter are published in two ways: publish to pose_kf topic and using tf to publish the pose relationship between /odom_kf and /base_footprint. Then all result and time stamp are recorded in two arrays and plot with matplotlib package after getting all the results.

## Performance and Evaluation
The result of pose and using Kalman Filter with different input can be seen as follows. They are all compared with the ground truth 1.1m as the final distance to the start pose.
![pose](https://github.com/sixer51/kalman_filter/blob/master/results/pose.png)  
Pose topic gets the second minimum error, but has obvious step changing behavior because it updates in a relative low rate(about 5hz, measured by rostopic hz pose)   
![goal_b](https://github.com/sixer51/kalman_filter/blob/master/results/goal_b.png)  
The result of kalman filter with scan and cmd_vel seems to get a good estimation. The final error is worse than pose and result using pose, but compare to our initial task, using cmd_vel to drive forward for 1m. This result obviously provides a better localization information.  
![goal_c](https://github.com/sixer51/kalman_filter/blob/master/results/goal_c.png)  
The result of kalman filter with scan and pose gets the minimum error. The first one second looks bad because the data from pose is still zero. The last two result proves that kalman filter with a good sensor can let us get better localization information.  
![goal_d](https://github.com/sixer51/kalman_filter/blob/master/results/goal_d.png)  
The result of kalman filter with camera_scan obviously get a much worse result than previous three. The main reason is that the scan message computed from depth image is totally not accurate. It contains a lot wrong data, so I usually have to use last data to replace a new but obviously wrong data.  
![goal_e](https://github.com/sixer51/kalman_filter/blob/master/results/goal_e.png)  
The result of kalman filter with pose and camera_scan get the worst result. The last two results let us see that kalman filter with a bad measurment can ruin the localization estimation.


## Referrence
For this assignment, I use the bag file recorded by Mingi Jeong. The one I recorded lack of /scan topic.
