Pegasus Drone v1.0.0
====================

CAD Model
---------
The lastest CAD model for the Pegasus Drone can be found in the `Pegasus CAD <https://github.com/PegasusResearch/pegasus_cad>`_ repository under a `Creative Commons Non-Commercial & Non-Military License <https://github.com/PegasusResearch/pegasus_cad/blob/main/LICENSE>`_. 

.. image:: https://github.com/PegasusResearch/pegasus_cad/blob/main/docs/_static/full_assembly.png?raw=true
  :width: 600
  :align: center
  :alt: Pegasus Drone CAD Model

Bill of Materials
-----------------

In order to replicate the Pegasus Drone, the following components are required:

* 1x `Nvidia Jetson Orin Nano Developer Kit <https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit>`_
* 1x 1Tb NVMe SSD PCIe 4.0
* 1x `Kakute H7 v1.3 stack (Microcontroller + 4in1 ESC) <https://holybro.com/products/kakute-h7-v1-stacks?variant=42833125277885>`_ 
* 4x `T-motor V2306 V2 2400KV <https://store.tmotor.com/product/v2306-v2-fpv-motor.html>`_
* 1x `Micro M8N GPS <https://holybro.com/collections/standard-gps-module/products/micro-m8n-gps>`_
* 1x `FrSky XM Plus ACCST 16CH Sbus <https://www.frsky-rc.com/product/xm-plus/>`_
* 1x `Realsense d435i <https://www.intelrealsense.com/depth-camera-d435i/>`_
* 4x `Propellers Dalprop Cyclone T5050 <http://www.dalprop.com>`_
* 1x Battery 4S 4300mAh
* 1x Set of jumper dupont wires
* 1x XT60 cable connector

PX4 Configuration for Indoor Flight
-----------------------------------

Flashing the Jetson Orin Nano
-----------------------------

In order to setup the Jetson Orin Nano Developer Kit, follow the instructions bellow. 

1. Start by installing the `Nvidia SDK manager <https://developer.nvidia.com/sdk-manager>`__ on a machine with Ubuntu 22.04LTS or later.
2. Connect the Jetson Orin Nano Developer Kit to the machine using a USB-C cable.
3. Short the FC REC and GND pins on the Jetson Orin Nano Developer Kit, under the SoC module.
4. Connect the Jetson Orin Nano Developer Kit to the power supply.

5. Before opening the Nvidia SDK Manager, make sure to turn off the Ubuntu Firewall (if enabled) by running the following command:

  .. code:: bash

      sudo ufw disable

6. Open the Nvidia SDK Manager and select the Jetson Orin Nano Developer Kit from the list of devices.

.. image:: /_static/jetson/sdk_manager_device.png
  :width: 600
  :align: center
  :alt: Nvidia SDK Manager Device selection

7. Select all the modules that can be installed and the latest jetpack version.

.. image:: /_static/jetson/sdk_manager_installing.png
  :width: 600
  :align: center
  :alt: Nvidia SDK Manager Modules selection

8. After the OS is installed, the Jetson Orin Nano Developer Kit will reboot and you will be prompted to enter the defined username and password for the installation to continue.

.. image:: /_static/jetson/sdk_manager_jetpack.png
  :width: 600
  :align: center
  :alt: Nvidia SDK Manager Jetpack installation

9. After this step is finished, you can unplug the Jetson Orin Nano Developer Kit from the power. Remove the short between the FC REC and GND pins and connect the Jetson Orin Nano Developer Kit to the power supply again.
Don't forget, it you disabled the Ubuntu Firewall, you should enable it again by running the following command:

  .. code:: bash

      sudo ufw enable

You can find more information on the `Nvidia Jetson Orin Nano Developer Kit <https://developer.nvidia.com/embedded/learn/get-started-jetson-orin-nano-devkit>`_ page.

Connecting the Jetson to Wifi
-----------------------------

1. After the Jetson reboots, an SSH connection is available through the USB-C connection. SSH into the Jetson Orin Nano using the following command:

  .. code:: bash

      ssh <username>@192.168.55.1

Replace the ``<username>`` with the username you defined during the installation process.

2. Connect the Jetson to a wifi network by running the following command:

  .. code:: bash

      sudo nmcli device wifi connect <SSID> password <password>

  .. admonition:: Wifi Connection

      Replace the ``<SSID>`` with the wifi network name and the ``<password>`` with the wifi network password. If you need to check the wifi networks available, you can run the following command:

      .. code:: bash

          nmcli device wifi list

      To check the connection status, you can run the following command:

      .. code:: bash

          nmcli device status 

Configuring the base software
-----------------------------

1. Install basic software packages by running the following command:

  .. code:: bash

      sudo apt update && sudo apt upgrade -y && sudo apt install -y htop tmux nano python3-pip && sudo apt autoremove

2. Remove "garbage" that comes pre-installed by default:

  .. code:: bash

      sudo apt purge thunderbird libreoffice-* firefox -y && sudo apt autoremove && sudo apt clean

3. Install Jetson Stats by running the following command:

  .. code:: bash

      sudo -H pip3 install -U jetson-stats && sudo systemctl restart jtop.service

4. Reboot the Jetson and SSH into the Jetson again. Run the following command:

  .. code:: bash

      sudo reboot

Installing ROS 2
----------------

1. Install ROS 2 Humble, by following the instructions on the `ROS 2 Humble <https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html>`_ page.

  .. code:: bash

      # Locale setup
      locale  # check for UTF-8

      sudo apt update && sudo apt install locales
      sudo locale-gen en_US en_US.UTF-8
      sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
      export LANG=en_US.UTF-8

      locale  # verify settings

      # Setup sources.list
      sudo apt install software-properties-common
      sudo add-apt-repository universe

      sudo apt update && sudo apt install curl -y
      sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

      echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

      sudo apt update
      sudo apt upgrade

      # Install ROS 2 packages
      sudo apt install ros-humble-desktop -y 
      sudo apt install ros-dev-tools -y 

      # Add the ROS 2 environment to the bashrc
      echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

Installing OpenCV with CUDA
---------------------------

1. Install OpenCV with CUDA support

  .. code:: bash

      # Remove old versions or previous builds
      cd ~ 
      sudo rm -rf opencv*

      # Download the latest version
      git clone --depth=1 https://github.com/opencv/opencv.git -b 4.10.0
      git clone --depth=1 https://github.com/opencv/opencv_contrib.git -b 4.10.0

      # reveal the CUDA location
      cd ~
      sudo sh -c "echo '/usr/local/cuda/lib64' >> /etc/ld.so.conf.d/nvidia-tegra.conf"
      sudo ldconfig
      
      # Create the build directory
      cd ~/opencv
      mkdir build
      cd build

      # Setup the architecture for cuda in the orin nano
      # Check: https://developer.nvidia.com/cuda-gpus#compute
      export ARCH=8.7
      export PTX="sm_87"

      # Install some dependencies
      sudo apt-get install -y libswresample-dev libdc1394-dev cmake libjpeg-dev libjpeg8-dev libjpeg-turbo8-dev libpng-dev libtiff-dev libglew-dev libavcodec-dev libavformat-dev libswscale-dev libgtk2.0-dev libgtk-3-dev libcanberra-gtk* libxvidcore-dev libx264-dev libtbb-dev libxine2-dev libv4l-dev v4l-utils qv4l2 libtesseract-dev libpostproc-dev libvorbis-dev libfaac-dev libmp3lame-dev libtheora-dev libopencore-amrnb-dev libopencore-amrwb-dev libopenblas-dev libatlas-base-dev libblas-dev liblapack-dev liblapacke-dev libeigen3-dev gfortran libhdf5-dev libprotobuf-dev protobuf-compiler libgoogle-glog-dev libgflags-dev

      # run cmake
      cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr \
      -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
      -D EIGEN_INCLUDE_PATH=/usr/include/eigen3 \
      -D WITH_OPENCL=OFF \
      -D CUDA_ARCH_BIN=${ARCH} \
      -D CUDA_ARCH_PTX=${PTX} \
      -D WITH_CUDA=ON \
      -D WITH_CUDNN=ON \
      -D WITH_CUBLAS=ON \
      -D ENABLE_FAST_MATH=ON \
      -D CUDA_FAST_MATH=ON \
      -D OPENCV_DNN_CUDA=ON \
      -D ENABLE_NEON=ON \
      -D WITH_QT=OFF \
      -D WITH_OPENMP=ON \
      -D BUILD_TIFF=ON \
      -D WITH_FFMPEG=ON \
      -D WITH_GSTREAMER=ON \
      -D WITH_TBB=ON \
      -D BUILD_TBB=ON \
      -D BUILD_TESTS=OFF \
      -D WITH_EIGEN=ON \
      -D WITH_V4L=ON \
      -D WITH_LIBV4L=ON \
      -D WITH_PROTOBUF=ON \
      -D OPENCV_ENABLE_NONFREE=ON \
      -D INSTALL_C_EXAMPLES=OFF \
      -D INSTALL_PYTHON_EXAMPLES=OFF \
      -D PYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages \
      -D OPENCV_GENERATE_PKGCONFIG=ON \
      -D BUILD_EXAMPLES=OFF \
      -D CMAKE_CXX_FLAGS="-march=native -mtune=native" \
      -D CMAKE_C_FLAGS="-march=native -mtune=native" ..
    
      # Compile the code
      make -j 6

      # Remove the old opencv installation
      sudo rm -rf /usr/include/opencv4/opencv2

      # Install the compiled library in the system
      sudo make install
      sudo ldconfig

      # Clean the compiled code fromt the build directory
      make clean
      sudo apt-get update
      sudo rm -rf opencv
      sudo rm -rf opencv_contrib


7. Setup the serial pins for communication with the microcontroller

  .. code:: bash

      sudo systemctl stop nvgetty.service
      sudo systemctl disable nvgetty.service
      sudo usermod -aG dialout marcelo

8. Remove the GNOME GUI (Optional)

  .. code:: bash

      sudo apt-get purge gnome-shell ubuntu-wallpapers-bionic light-themes chromium-browser* libvisionworks libvisionworks-sfm-dev -y
      sudo apt-get autoremove -y
      sudo apt clean -y

Realsense Setup
---------------

