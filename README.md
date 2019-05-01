# ROS HPS Camera node

This is a tentative of a ROS package to handle the HPS-3D Lidar.
It is based on the messy code and document given by Seedstudio and also this repository <https://github.com/ropod-project/hps_camera> (which kind of works but only in single shot mode)

You need to copy the proprietary driver library (in lib) to /usr/local/lib (don't forget to sudo ldconfig)
There is a 32bit and a 64bit version.
