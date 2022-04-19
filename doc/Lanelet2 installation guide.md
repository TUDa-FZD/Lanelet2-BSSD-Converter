# Installation guide for a Lanelet2 Workspace with Catkin/ROS

## Prerequisites
Lanelet2 on [Github](https://github.com/fzi-forschungszentrum-informatik/Lanelet2)  
OS used for this tutorial: Ubuntu 20.04

## Installation Guide (Python options mentioned within the steps)
1. Follow the [installation guide for ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
   1. Follow 1.1 – 1.4 as described 
      1. Use the desktop-full version for instance 
   2. skip 1.5 
   3. 1.6 as described
2. Install dependencies in terminal: 
   1. invoke  
      <code>sudo apt-get install ros-noetic-rospack ros-noetic-catkin ros-noetic-mrt-cmake-modules</code>  
   2. and invoke 
      <code>sudo apt install python3-osrf-pycommon python3-catkin-tools</code>
3. Setup catkin workspace with Lanelet2 
   1. Create a new folder on your system for the catkin workspace (name it “Lanelet2_WS” for instance)
   2. Create a folder “src” within the folder created at 3.1 
   3. Open a terminal in “src” and invoke  
<code>git clone https://github.com/fzi-forschungszentrum-informatik/lanelet2.git </code>  
   4. invoke <code>cd ..</code> to get to the enclosing directory 
   5. invoke
<code>rosdep install --from-paths src --ignore-src --rosdistro noetic </code>
   6. For Python support, do the following steps:
      1. Make sure you have venv (May require root):  
         <code>apt-get install python3-venv</code>
      2. Create a virtual environment in the catkin workspace folder:  
         <code>python3 -m venv env</code>
   7. in "Lanelet2_WS" directory invoke  
      <code>catkin init</code>  
      and for Python setup (edit the python version at the end to adjust to the version on your system! Ubuntu 20.04 ships with 3.8)  
      <code>catkin config --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DPYTHON_VERSION=3.8 </code>
   8. in "Lanelet2_WS" directory invoke  
      <code>catkin build</code>
   9. add the following code to to ~/.bashrc (In Files, go to home, enable “show hidden files” in the hamburger menu and
      add those two lines at the very end of the file “.bashrc”, save and close)
       <code>source /opt/ros/noetic/setup.bash  
       source "your path to the workspace-folder created in 3.1"/devel/setup.bash </code>
4. Test your installation: 
   1. Open Terminal anywhere 
   2. type, but don’t invoke
      <code>rosrun lane </code>
   3. Press tab on your keyboard
   4. If the word “lane” is automatically completed to “lanelet2”, the installation worked.


## Error-Guide
If the catkin build command throws an error (“Can’t find boost_python3”), check
[this](https://github.com/fzi-forschungszentrum-informatik/Lanelet2/issues/221)
out (In short: Replace the file
<code>/opt/ros/noetic/share/mrt_cmake_modules/cmake/Modules/FindBoostPython.cmake</code>
with the one from
[this link](https://github.com/KIT-MRT/mrt_cmake_modules/blob/master/cmake/Modules/FindBoostPython.cmake).