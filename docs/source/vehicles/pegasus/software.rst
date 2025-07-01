Software Setup
==============

In order to get your Pegasus drone ready to fly, you need to setup the software stack. This includes installing the necessary software packages, setting up the network, and configuring the GPIO pins. It also includes flashing the Kakute H7 microcontroller with PX4.

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

7. Select all the modules that can be installed and the latest jetpack version. Setup the ``Pre-config`` option and set the username (for example ``pegasus``) and password, and continue.

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

Passwordless SSH
----------------

To enable passwordless SSH, follow the instructions bellow.

1. Generate an SSH key on your computer and copy it to the remote machine:

  .. code:: bash

      # Generate the SSH key
      ssh-keygen -t ed25519 -C "your_email@example.com" -f ~/.ssh/pegasus

      # Copy the SSH key to the remote machine
      ssh-copy-id -i ~/.ssh/pegasus.pub <drone_username>@<drone_ip>

      # Test the connection
      ssh <drone_user>@<drone_ip>

2. Add the following to your computer ~/.ssh/config file:

  .. code:: bash

      Host <Hostname (example: pegasus)>
        HostName drone_ip
        User <drone_user>
        IdentityFile ~/.ssh/pegasus
        AddKeysToAgent yes
        IdentitiesOnly yes

3. Now you should be able to SSH into the Jetson without the need for a password:

  .. code:: bash

      ssh <Hostname>

Removing Pre-installed Software
-------------------------------

1. Install basic software packages by running the following command:

  .. code:: bash

      sudo apt update && sudo apt upgrade -y && sudo apt install -y htop tmux nano python3-pip && sudo apt autoremove

2. Remove "garbage" that comes pre-installed by default:

  .. code:: bash

      sudo apt purge thunderbird libreoffice-* onboard aisleriot gnome-sudoku gnome-mines gnome-mahjongg cheese gnome-calculator gnome-todo shotwell gnome-calendar rhythmbox simple-scan remmina transmission-gtk -y && sudo apt autoremove && sudo apt clean

3. Install Jetson Stats by running the following command:

  .. code:: bash

      sudo -H pip3 install -U jetson-stats && sudo systemctl restart jtop.service

4. Reboot the Jetson and SSH into the Jetson again. Run the following command:

  .. code:: bash

      sudo reboot

5. Run Jetson Stats and set the power mode to MAXN (for maximum performance):

  .. code:: bash

      jtop


Installing NVIDIA Video Codec SDK
----------------------------------

To enable video processing with GPU acceleration, you need to install the NVIDIA Video Codec SDK. Follow the instructions bellow:

1. Download the latest NVIDIA Video Codec from ``https://developer.nvidia.com/nvidia-video-codec-sdk/download``.

  .. code:: bash

      # Download the video codec sdk
      cd ~
      wget https://developer.nvidia.com/downloads/designworks/video-codec-sdk/secure/13.0.19/video_codec_sdk_13.0.19.zip
      unzip video_codec_sdk_13.0.19.zip

2. Copy the library into the cuda library directory

  .. code:: bash

      # Copy the library to the cuda directory
      sudo cp ~/Video_Codec_SDK_13.0.19/Lib/linux/stubs/aarch64/libnvcuvid.so /usr/local/cuda/lib64/libnvcuvid.so
      sudo cp ~/Video_Codec_SDK_13.0.19/Lib/linux/stubs/aarch64/libnvidia-encode.so /usr/local/cuda/lib64/libnvidia-encode.so

      # Copy the header files to the cuda directory
      sudo cp -s ~/Video_Codec_SDK_13.0.19/Interface/cuviddec.h /usr/local/cuda/include/cuviddec.h
      sudo cp -s ~/Video_Codec_SDK_13.0.19/Interface/nvcuvid.h /usr/local/cuda/include/nvcuvid.h
      sudo cp -s ~/Video_Codec_SDK_13.0.19/Interface/nvEncodeAPI.h /usr/local/cuda/include/nvEncodeAPI.h

      # Update the linking libraries
      sudo ldconfig


Installing OpenCV with GPU Acceleration
---------------------------------------

For some reason, the OpenCV that comes pre-installed in the Jetson is not compiled with CUDA support enabled, nor video processing with GPU acceleration. To enable CUDA support, follow the instructions bellow.


1. Install OpenCV with CUDA support

  .. code:: bash

      # Remove the old opencv installation
      sudo rm -rf /usr/include/opencv4/opencv2
      sudo apt purge -y *libopencv*

      # Go into the home directory
      cd ~ 

      # Download the latest version
      git clone --depth=1 https://github.com/opencv/opencv.git -b 4.11.0
      git clone --depth=1 https://github.com/opencv/opencv_contrib.git -b 4.11.0

      # reveal the CUDA location
      cd ~
      sudo sh -c "echo '/usr/local/cuda/lib64' >> /etc/ld.so.conf.d/nvidia-tegra.conf"
      sudo ldconfig
      
      # Create the build directory
      cd ~/opencv
      mkdir build
      cd build

      # Setup the architecture for cuda in the jetson orin nano
      # Check the version for your board in: https://developer.nvidia.com/cuda-gpus#compute
      export ARCH=8.7
      export PTX="sm_87"

      # Install mathematic libraries
      sudo apt-get install -y libopenblas-dev libopenblas-base libatlas-base-dev liblapacke-dev

      # Install some dependencies
      sudo apt-get install -y libswresample-dev libdc1394-dev cmake libjpeg-dev libjpeg8-dev libjpeg-turbo8-dev libpng-dev libtiff-dev libglew-dev libavcodec-dev libavformat-dev libswscale-dev libgtk2.0-dev libgtk-3-dev libcanberra-gtk* libxvidcore-dev libx264-dev libtbb-dev libxine2-dev libv4l-dev v4l-utils qv4l2 libtesseract-dev libpostproc-dev libvorbis-dev libfaac-dev libmp3lame-dev libtheora-dev libopencore-amrnb-dev libopencore-amrwb-dev libopenblas-dev libatlas-base-dev libblas-dev liblapack-dev liblapacke-dev libeigen3-dev gfortran libhdf5-dev libprotobuf-dev protobuf-compiler libgoogle-glog-dev libgflags-dev

      # run cmake (with as much CUDA acceleration as we can)
      cmake -D CMAKE_BUILD_TYPE=RELEASE \
      -D CMAKE_INSTALL_PREFIX=/usr \
      -D OPENCV_EXTRA_MODULES_PATH=~/opencv_contrib/modules \
      -D EIGEN_INCLUDE_PATH=/usr/include/eigen3 \
      -D WITH_OPENCL=OFF \
      -D CUDA_ARCH_BIN=${ARCH} \
      -D CUDA_ARCH_PTX=${PTX} \
      -D WITH_CUDA=ON \
      -D WITH_CUDNN=ON \
      -D WITH_NVCUVID=ON \
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
      -D JPEG_INCLUDE_DIR=/usr/include \
      -D JPEG_LIBRARY=/usr/lib/aarch64-linux-gnu/libjpeg.a \
      -D CMAKE_CXX_FLAGS="-march=native -mtune=native" \
      -D CMAKE_C_FLAGS="-march=native -mtune=native" ..
    
      # Compile the code
      make -j$(nproc)

  .. admonition:: Warning
  
    This is the time where you should go grab a cup of coffe and wait for the compilation to finish. If the compilation fails due to NVIDIA Video Codec SDK, this is because OpenCV has a bug yet to be fixed regarding the latest version of the NVIDIA SDK. Just go to the file where the compilation is failing and replace the lines that are causing the error with the option provided bellow.


2. Install the compiled library in the system

  .. code:: bash

      # Install the compiled library in the system
      sudo make install
      sudo ldconfig

      # Clean the compiled code fromt the build directory
      make clean
      sudo apt-get update

3. Test the installation

  .. code:: bash

    # Open python3 in the terminal
    python3

    # Run the following code
    import cv2
    count = cv2.cuda.getCudaEnabledDeviceCount()
    print(count)


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

Setting up the GPIO pins
------------------------

To setup the serial pins for communication with the microcontroller, follow the instructions bellow:

  .. code:: bash

      sudo systemctl stop nvgetty.service
      sudo systemctl disable nvgetty.service
      sudo usermod -aG dialout <username>

Disabling GUI
-------------

For a better performance, it is recommended to disable the GNOME GUI. To do so, follow the instructions bellow:

  .. code:: bash

      # Setup the system to boot in text mode
      sudo systemctl set-default multi-user.target

.. admonition:: Optional

  Alternatively, you can remove the GUI packages altogether by running the following lines.

  .. code:: bash

      # Remove the GNOME GUI
      sudo apt-get purge gnome-shell ubuntu-wallpapers-bionic light-themes chromium-browser* libvisionworks libvisionworks-sfm-dev -y
      sudo apt-get autoremove -y
      sudo apt clean -y

      # Setup the system to boot in text mode
      sudo systemctl set-default multi-user.target

  In practice, we just disable the GUI but keep the packages installed in case we need to use the GUI in the future.


Realsense Setup
---------------

The next step is to install the realsense library for the Intel RealSense cameras. Follow the instructions bellow.

  .. code:: bash

      # Go to the home directory
      cd ~

      # Install some dependencies
      sudo apt-get install git cmake libssl-dev freeglut3-dev libusb-1.0-0-dev pkg-config libgtk-3-dev unzip -y

      # Clone the correct version of the library that matches the ROS 2 version that we are using in Pegasus
      git clone https://github.com/IntelRealSense/librealsense.git -b v2.56.3

      # Create the build directory where the code will be compiled
      cd librealsense
      mkdir librealsense_build && cd librealsense_build

      # Setup the udev rules for the driver to be able to access the camera data
      sudo cp ../config/99-realsense-libusb.rules /etc/udev/rules.d/
      sudo udevadm control --reload-rules && udevadm trigger

      # Setup the architecture for cuda in the orin nano
      # Check: https://developer.nvidia.com/cuda-gpus#compute
      export ARCH=8.7

      # Compile the code with CUDA support and python bindings
      cmake ../ -DBUILD_EXAMPLES=true -DFORCE_RSUSB_BACKEND=true -DBUILD_WITH_CUDA=true -DCMAKE_BUILD_TYPE=release -DBUILD_PYTHON_BINDINGS=bool:true -DPYTHON_EXECUTABLE=/usr/bin/python3
      make -j$(nproc)
      
      # Install the library in the system
      sudo make install
      sudo ldconfig

      # Reboot the PC - The next time you boot, the intel realsense camera should be working
      sudo reboot

      # Test the camera (should still be laggy, but it is ok for now, dont worry)
      realsense-viewer


Setup a Github SSH Key
----------------------

To `setup an SSH key for your Github account <https://docs.github.com/en/authentication/connecting-to-github-with-ssh/generating-a-new-ssh-key-and-adding-it-to-the-ssh-agent>`_, follow the instructions bellow.

1. Generate an SSH key on the Jetson computer:

  .. code:: bash

    # Generate the SSH key
    ssh-keygen -t ed25519 -C "your_email@example.com" -f ~/.ssh/github

    # Start the ssh agent in the background
    eval "$(ssh-agent -s)"

    # Add your key to the agent
    ssh-add ~/.ssh/github

2. Add the following to the jetson ~/.ssh/config file:

  .. code:: bash

    Host github.com
      HostName github.com
      User git
      IdentityFile ~/.ssh/github
      AddKeysToAgent yes
      IdentitiesOnly yes
  
3. Copy the public SSH key to your github account. To do so, go to your Github account settings and add the public key to the SSH keys section.


Installing Pegasus GNC
----------------------

1. Start by installing some dependencies

  .. code:: bash

    # Install the dependencies for CasADi (IPOPT)
    sudo apt install -y coinor-libipopt-dev python3-pip

    # Install dependencies for the Realsense ROS 2 package
    sudo apt install -y ros-humble-image-transport ros-humble-diagnostic-updater

    # Install dependencies for the video server
    sudo apt install -y ros-humble-async-web-server-cpp

2. Clone the Pegasus repository that contains the GNC code and configurations for the Pegasus vehicle built in.

  .. code:: bash

    # Go to the home directory
    cd ~

    # Clone the Pegasus repository that contains the GNC code + configurations for the drone itself
    git clone git@github.com:PegasusResearch/pegasus_drone.git --recursive


3. Compile the Pegasus external code

  .. code:: bash

    # Compile the Pegasus external code
    cd pegasus_drone/pegasus_external
    colcon build --symlink-install

    # --------------------------------------------------------------------
    # Run the following command to add the ROS 2 environment to the bashrc
    # --------------------------------------------------------------------

    # Add the ROS 2 environment to the bashrc if not already
    echo "source $HOME/pegasus_drone/pegasus_external/install/setup.bash" >> ~/.bashrc

    # Add an alias to the bashrc for automatically source new terminals
    echo "alias s='source $HOME/.bashrc'" >> ~/.bashrc

    # source your updated bashrc
    source ~/.bashrc

4. Compile the Pegasus GNC code
  
  .. code:: bash

    # Go to the Pegasus GNC code
    cd ~/pegasus_drone/pegasus

    # Compile the Pegasus GNC code
    colcon build --symlink-install

    # Add the ROS 2 environment to the bashrc if not already
    echo "source $HOME/pegasus_drone/pegasus/install/setup.bash" >> ~/.bashrc

5. Run the config.bash script that will install services to automatically start the jetson in the maximum power mode and start the Pegasus GNC code at boot.

  .. code:: bash

    # Run the config.bash script
    cd ~/pegasus_drone/pegasus/src/pegasus_drone/scripts
    sudo ./config.bash


Installing Machine Learning Libraries
-------------------------------------

**1. Installing Pytorch**

  .. code:: bash

      # https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html

      # Install CUSparse for accelerating computations
      wget https://developer.download.nvidia.com/compute/cusparselt/redist/libcusparse_lt/linux-aarch64/libcusparse_lt-linux-aarch64-0.6.3.2-archive.tar.xz
      tar xf libcusparse_lt-linux-aarch64-0.6.3.2-archive.tar.xz
      sudo cp -a libcusparse_lt-linux-aarch64-0.6.3.2-archive/include/* /usr/local/cuda/include/
      sudo cp -a libcusparse_lt-linux-aarch64-0.6.3.2-archive/lib/* /usr/local/cuda/lib64/
      sudo ldconfig

      # Check the versions + links for pytorch + tensorimage stuff
      https://forums.developer.nvidia.com/t/pytorch-for-jetson/72048
      export JP_VERSION=61
      export PYT_VERSION=torch-2.5.0a0+872d972e41.nv24.08.17622132-cp310-cp310-linux_aarch64.whl
      export TORCH_INSTALL=https://developer.download.nvidia.com/compute/redist/jp/v$JP_VERSION/pytorch/$PYT_VERSION
      python3 -m pip install --upgrade pip
      python3 -m pip install numpy
      python3 -m pip install --no-cache $TORCH_INSTALL

      # Install the corresponding torch vision (see table in their repo)
      git clone https://github.com/pytorch/vision torchvision
      cd torchvision/
      git checkout v0.20.1
      export USE_CUDA=1 USE_CUDNN=1 USE_MKLDNN=1 TORCH_CUDA_ARCH_LIST="8.6" FORCE_CUDA=1 FORCE_MPS=1
      sudo apt-get -y install ffmpeg libavutil-dev libavcodec-dev libavformat-dev libavdevice-dev libavfilter-dev libswscale-dev libswresample-dev libswresample-dev libpostproc-dev libjpeg-dev libpng-dev libopenblas-base libopenmpi-dev
      python3 setup.py develop --user

**2. Installing Tensorflow**

  .. code:: bash

      https://docs.nvidia.com/deeplearning/frameworks/install-tf-jetson-platform/index.html#overview
      export JP_VERSION=60
      export TF_VERSION=tensorflow-2.15.0+nv24.05-cp310-cp310-linux_aarch64.whl
      export TENSORFLOW_INSTALL=https://developer.download.nvidia.com/compute/redist/jp/v$JP_VERSION/tensorflow/$TF_VERSION
      sudo apt-get install libhdf5-serial-dev hdf5-tools libhdf5-dev zlib1g-dev zip libjpeg8-dev liblapack-dev libblas-dev gfortran
      python3 -m pip install -U testresources setuptools numpy future mock keras_preprocessing keras_applications gast protobuf pybind11 cython pkgconfig packaging h5py
      python3 -m pip install --no-cache $TENSORFLOW_INSTALL

Installing NVSlam (for NVIDIA Jetson Orin Nano)
-----------------------------------------------

NVSlam is a library for visual SLAM (Simultaneous Localization and Mapping) that is optimized for NVIDIA Jetson devices. It provides a high-performance solution for visual SLAM applications.

  .. code:: bash
    
      # Go to the home directory
      cd ~

      # Clone the Isaac ROS common repository and compile it
      git clone -b release-3.2 https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_common.git isaac_ros_common
      cd isaac_ros_common
      colcon build --symlink-install

      # Add the isaac-ros CDN
      wget -qO - https://isaac.download.nvidia.com/isaac-ros/repos.key | sudo apt-key add -
      grep -qxF "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" /etc/apt/sources.list || \
      echo "deb https://isaac.download.nvidia.com/isaac-ros/release-3 $(lsb_release -cs) release-3.0" | sudo tee -a /etc/apt/sources.list
      sudo apt-get update

      sudo apt update && sudo apt install curl -y
      sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

      echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

      # Install the NVSlam ROS2 package
      sudo apt-get install -y ros-humble-isaac-ros-visual-slam


Network Setup
-------------

.. list-table:: ID, IPs and Mavlink Ports and other information
   :widths: 5 15 15 10 10 10 10 10
   :header-rows: 1
    
   * - ID
     - Station SSID
     - Station IP
     - Forward Address
     - Status
     - Kakute version
     - Config File Indoor (Mocap)
     - Config File Outdoor (GPS)
   * - Personal
     - Quadrotor
     - 192.168.1.166
     - 192.168.1.232:15000
     - ✔️
     - KakuteH7v1.3-mini
     - pegasus.params
     - pegasus.params
   * - 1
     - Quadrotor
     - 192.168.1.241
     - 192.168.1.232:15001
     - ❌
     - KakuteH7v1.3
     - TODO
     - TODO
   * - 2
     - Quadrotor
     - 192.168.1.242
     - 192.168.1.232:15002
     - ❌
     - KakuteH7v1.3
     - TODO
     - TODO