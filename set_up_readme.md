# OMRE-SIUE
6 Autonomous Mobile Robots at SIUE

Install Jetpack: (fixed error)
sudo dpkg -i /home/nvidia/cuda-l4t/cuda-repo-l4t-9-0-local_9.0.252-1_arm64.deb
https://devtalk.nvidia.com/default/topic/1036338/jetson-tx1/unable-to-complete-post-installation-of-jetpack-3-2-on-tx1-cuda-9-0-installation-fails/

For jetpack 4.2 or higher:
https://devtalk.nvidia.com/default/topic/1050477/jetson-tx2/jetpack-4-2-flashing-issues-and-how-to-resolve/
sudo ./flash.sh jetson-tx2 mmcblk0p1 // For Jetson TX2

Install Arduino:
https://www.instructables.com/id/Install-Arduino-IDE-182-on-Linux/

Install Geany:
http://linuxpitstop.com/install-geany-1-25-on-ubuntu-15-04/
https://github.com/geany/geany-themes

Install OpenCV:
https://shiroku.net/robotics/install-opencv-on-jetson-tx2/

Install Intel_RealSense
https://www.robotexchange.io/t/guide-install-librealsense-l-ubuntu-16/218

update pip:
https://askubuntu.com/questions/642533/python-pip-broken-on-ubuntu
https://askubuntu.com/questions/1025793/running-pip3-importerror-cannot-import-name-main

pip error:
$ hash -d pip

cairo error:
sudo apt-get install python-gi-cairo

cmake:
https://devtalk.nvidia.com/default/topic/1048533/how-to-install-cmake-3-8-in-jetson-tx2-/
