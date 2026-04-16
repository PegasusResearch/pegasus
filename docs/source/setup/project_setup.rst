Project Setup
=============

1. Install TensorRT using the .deb packages from (https://docs.nvidia.com/deeplearning/tensorrt/latest/installing-tensorrt/install-debian.html).

    1.1 https://developer.nvidia.com/tensorrt/downloads
    1.2 Select the appropriate version for your system.
    1.3 ```sudo dpkg -i nv-tensorrt-local-repo-ubuntu2404-10.x.x-cuda-x.x_1.0-1_amd64.deb```
    1.4 ```sudo cp /var/nv-tensorrt-local-repo-ubuntu2404-10.x.x-cuda-x.x/*-keyring.gpg /usr/share/keyrings/```
    1.5 ```sudo apt-get update```
    1.6 ```sudo apt-get install tensorrt```


2. Compile OpenCV with CUDA and TensorRT support.
```
# Remove old OpenCV headers
sudo rm -rf /usr/include/opencv4

# Remove old OpenCV libraries
sudo rm -f /usr/lib/x86_64-linux-gnu/libopencv_*
sudo rm -f /usr/lib/x86_64-linux-gnu/cmake/opencv4 -rf

# Remove old pkg-config file
sudo rm -f /usr/lib/x86_64-linux-gnu/pkgconfig/opencv4.pc

# Then install the new build
sudo make install
sudo ldconfig  # refresh the dynamic linker cache

sudo apt remove --purge libopencv* python3-opencv
sudo apt autoremove

# Install requirements for OpenCV build
sudo apt install qt6-base-dev libqt6opengl6-dev libgtk2.0-dev libgtk-3-dev pkg-config

# Go into the home directory
cd ~

# Download the latest version
git clone --depth=1 https://github.com/opencv/opencv.git -b 4.13.0
git clone --depth=1 https://github.com/opencv/opencv_contrib.git -b 4.13.0

# Create the build directory
cd ~/opencv
mkdir build
cd build

# Setup the architecture for cuda, which depends on your graphics card Version. For example, the 5000 seris is 12.0
# Check the version for your board in: https://developer.nvidia.com/cuda-gpus#compute
export ARCH=12.0
export PTX="sm_120"

cmake \
  -D CMAKE_BUILD_TYPE=RELEASE \
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
  -D ENABLE_NEON=OFF \
  -D WITH_QT=ON \
  -D WITH_OPENGL=ON \
  -D WITH_GTK=ON \
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
  -D INSTALL_PYTHON_EXAMPLES=ON \
  -D PYTHON3_PACKAGES_PATH=/usr/lib/python3/dist-packages \
  -D OPENCV_GENERATE_PKGCONFIG=ON \
  -D BUILD_EXAMPLES=ON \
  -D BUILD_opencv_highgui=ON \
  -D BUILD_opencv_cvv=OFF \
  -D BUILD_opencv_rgbd=OFF \
  -D CMAKE_CXX_FLAGS="-march=native -mtune=native" \
  -D CMAKE_C_FLAGS="-march=native -mtune=native" \
  ..

make -j$(nproc)
sudo make install
sudo ldconfig
```


3. Generating a TensorRT model from an ONNX model
```
/usr/src/tensorrt/bin/trtexec \
  --onnx=yolov8n-pose.onnx \
  --saveEngine=yolov8n-pose.engine \
  --fp16 \
  --inputIOFormats=fp16:chw \
  --outputIOFormats=fp16:chw
```