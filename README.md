# initial_robot_pose_ARtag

<p align="justify">
Localization is a key requirement for autonomous navigation. While AMCL <i>(Adaptive Monte Carlo Localization)</i> helps robots localize on a known map, it requires an <b>initial pose estimate</b> to start converging. Traditionally, this estimate is provided manually via Rviz or configuration files, but this can be slow and inaccurate.
</p>
<p align="justify">
This project introduces a <b>vision-based method using AR tags</b> to automatically set the robot's initial pose in ROS. It detects an AR tag, computes the robot's pose relative to the map, and publishes it to <code>/initialpose</code>. This speeds up localization and reduces user intervention.
</p>

---

### ✨ Features
✅ Automatic computation of **initial pose** using AR tags.  
✅ Works regardles of AR tag position/orientation relative to the robot.  
✅ Publishes pose to `/initialpose` for AMCL to use  
✅ Includes a custom **ROS service (`amcl_pose_server`)** to query robot's estimated position.  

---

### 📂 Package Structure
```bash
mobile_robot/
├── src/  
│   ├── config/         
|   │   ├── costmap_common_params.yaml      # Defines shared costmap parameters  
|   │   ├── costmap_global_params.yaml      # Configures the global costmap for long-term path planning
|   │   ├── costmap_local_params.yaml       # Configures the local costmap for real-time obstacle avoidance  
|   │   ├── ekf_localization                # Sets parameters for sensor fusion using an extended kalman filter
|   │   ├── mobile_robot.rviz              # stores rviz visualization settings for the robot and its environment
|   │   └── planner.yaml                   # Defines path planning parameters
│   
│   ├── launch/            
|   │   ├── ar_marker_detech.launch         # Detect AR tags & compute poses wrt base link  
|   │   ├── husky_robot.launch              # Launch the Gazebo world, spawn the husky robot, start the controllers, start static trasform publisher, start the AR-tag detector
|   │   ├── mobile_robot_mapping.launch     # starts the slam_gmapping node for SLAM  
|   │   ├── mobile_robot_nav.launch         #load map & launch map server, launch amcl and move_base
|   │   └── target_pose_transform.launch    # Static transform of AR tag frames (linear & rotational) 
│
│   ├── scripts/            
|   │   ├── amclPoseServer.py               # Starts the amcl_pose_server service, returns the robot's pose estimate
|   │   └── move_goal.py                    # the python script to localize the robot
│
│   ├── world/            
|   │   └── custom_world.world              # custom gazebo world file
│
│   ├── xacro/             
|   │   ├── mobile_robot.xacro              # combines the Husky base with a kinect camera mounted on it
|   │   └── multi_kinect.xacro              # Define multiple Kinect camera models  
│
├── srv/                                  
│   └── amclPose.srv                        # amclPose service interface
│
├── CMakeLists.txt                            
└── package.xml

```

---

### 📥 Installation
Clone the repository into your ROS workspace:
```bash
cd ~/catkin_ws/src
git clone https://github.com/Joshua-Ayanlade/initial_robot_pose_ARtag.git
cd ..
catkin_make
source devel/setup.bash
```

---
### ⚙️ Running the System
   
1. Launch the Gazebo world, spawn the husky robot, start the controllers, start static trasform publisher, start the AR-tag detector:  
   `roslaunch mobile_robot husky_robot.launch`

2. Launch the amcl pose server:  
   `rosrun mobile_robot amclPoseServer.py`

3. Execute the python script to localize the robot:
   
   `rosrun mobile_robot move_goal.py 3`&nbsp;&nbsp;&nbsp;&nbsp;# *Estimate robot pose using AR Tag 3* 

---
### 🎥 Demo
![Setting+Initial+Pose](https://github.com/user-attachments/assets/80a1af2f-8d2e-4e95-973e-8223f586e352)


---
### 📊 Results
- Faster AMCL convergence compared to manual pose estimation.
- Tested with different AR tag orientations and positions – successful re-localization achieved.
- Enables dynamic re-localization before missions for improved goal accuracy.
  
---
### Acknowledgement
- husky_control package authored by **Paul Bovbel**
- husky_description package authored by **Ryan Gariepy et al**
