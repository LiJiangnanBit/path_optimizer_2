# path_optimizer_2
![](https://img.shields.io/badge/ROS-Kinetic-orange)  ![](https://img.shields.io/badge/ROS-Melodic-blue)     
**This is a newer version of my path planning ROS package for autonomous driving vehicles 👉[path_optimizer](https://github.com/LiJiangnanBit/path_optimizer).**  
![haha.png](https://i.loli.net/2021/11/02/VQLH5eRFmTDcgBM.png)  
## Improvements
- Higher success rate in dense/complicated environments.  
- Simpler problem formulation.  
- Candidate result when no collision-free solution exists.  
## Simulation
[(1) Simulation in dynamic environment](https://vimeo.com/498950818)  
[(2) Simulation with complex static obstacles](https://vimeo.com/498591477)  
## Dependencies  
<!-- :atom: TO BE UPDATED.  
For now, refer to 👉[path_optimizer](https://github.com/LiJiangnanBit/path_optimizer). It's generally the same.   -->  
- ROS Kinetic (Ubuntu 16.04) or Melodic (Ubuntu 18.04);  
- Other dependencies: [glog](https://github.com/google/glog),  [gflags](https://github.com/gflags/gflags), [osqp-eigen](https://github.com/robotology/osqp-eigen), [grid_map](https://github.com/ANYbotics/grid_map)  
-  Put these ROS packages in your ros workspace:  [ros_viz_tools](https://github.com/Magic-wei/ros_viz_tools), [tinyspline_ros](https://github.com/qutas/tinyspline_ros).  
These dependencies (except for ROS) can be installed by running script/install_deps.sh.  
## Usage
A png image is loaded as the grid map. You can click to specify the global reference path and the start/goal state of the vehicle.  
~~~
roslaunch path_optimizer_2 demo.launch
~~~
#### (1) Pick reference points using "Publish Point" tool in RViz.  
- Pick at least six points.  
- There are no hard and fast rules about the spacing of the points.  
- If you want to abandon the chosen points, just double click anywhere when using the "Publish Point" tool.  
- You can replace `gridmap.png` with other black and white images. Note that the resolution in `demo.cpp` is set to 0.2m, whick means that the length of one pixel is 0.2m on the map.  
- In application, the reference path is given by a global path or by a search algorithm like A*.  

![选点.gif](https://i.loli.net/2020/04/12/kRItwQTh5GJWHxV.gif)  
#### (2) Pick start state using "2D Pose Estimate" tool and pick goal state using "2D Nav Goal" tool.  
- Currently, it's not strictly required to reach the goal state. But this can be changed.    
- The start state must be ahead of the first reference point.  

![规划.gif](https://i.loli.net/2020/04/12/XmxgwTGRI1MtoVK.gif)  
