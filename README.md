# Final Assignment: Autonomous Driving
<center> Chenghao Xu (5266068, c.xu-7@student.tudelft.nl) </center>
<center> Liangchen Sui (5440238, l.sui@student.tudelft.nl) </center>

***
## Contribution
Chenghao Xu:  work on `opencv_person_detector`

Liangchen SUi: work on `pcl_obstacle_detector`
## How to build
Assuming that you do not work on the directory `~/catkin_ws` with singularity, the following commands have to be run in your terminal
```shell
source /opt/ros/melodic/setup.bash
cd ~
mkdir -p catkin_ws/src
cd catkin_ws/src
catkin_init_workspace
git clone git@gitlab.ro47003.3me.tudelft.nl:students-2122/lab4/group08.git
git clone git@gitlab.ro47003.3me.tudelft.nl:students-2122/ro47003_simulator.git
cd ..
catkin_make
source devel/setup.bash
```

## How to run
```
roslaunch control_barrel_world solution.launch
```


## Person detection package
###  **Class `CPersonDetector`**

**Private Members:**
| Type | name |
| -------------------- | ------------------ |
| ros::NodeHandle | nh|
| image_transport::Subscriber | imageSub |
| image_transport::Publisher | imagePub |
| ros::Publisher | msgPub |
| std::vector<cv::Rect> | detections_ |
| double | hit_threshold_ |
| double | stride_value_ |
| cv::Size | win_stride_ |


**Public Member Functions:**
| function | description |
| -------------------------- | ----------------------------------------------------------- |
| CPersonDetector() |  constructor |
| imageCallback() | callback function |
| imageDetecction() | HOG detection function |
| detectionsPublisher() | detection publisher function |
| ~CPersonDetector()  | destructor |

## Obstacle detection package
###  **Class `CObstacleDetector`**
**Private Members:**
| Type | name |
| -------------------- | ------------------ |
| ros::NodeHandle | nh|
| ros::Subscriber | pclSub |
| ros::Publisher | pclPub |
| double | distance_threshold_ |
| double | cluster_tolerance_ |
| double | max_cluster_size_ |


**Public Member Functions:**
| function | description |
| -------------------------- | ----------------------------------------------------------- |
| CObstacleDetector() |  constructor |
| pclCallback() | callback function |
| GroundPlaneRemoval() |removing ground plane from point cloud |
| clusterExtraction() | extract the cluster from filtered point cloud |
| clusterPublisher() | publish the clusters |
| ~CObstacleDetector()  | destructor |



## Prius control package
###  **Class `CPriusControl`**
**Private Members:**
| Type | name |
| -------------------- | ------------------ |
| ros::NodeHandle | nh|
| ros::Subscriber | personDetectorSub |
| ros::Subscriber | obstacleDetectorSub |
| ros::Publisher | controlPub |
| bool | personDetector_|
| bool | obstacleDetector_ |
| float | steer_ |


**Public Member Functions:**
| function | description |
| -------------------------- | ----------------------------------------------------------- |
| CPriusControl() |  constructor |
| personDetectorCallback() | callback function |
| obstacleDetectorCallback() | callback function |
| controlPublish() | publish the control message |
| getYValue() | get the closest y-value of the obstacle |
| ~CPriusControl()  | destructor |
