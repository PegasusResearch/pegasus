# vrpn_client_ros
This for aims at porting the original code from the kinetic-devel branch to ROS2.

## What works?

I only use pose in my project, so I did not port anything else (TF, twist, accel). Also multiple sensors per tracker are not ported.
If there is anyone who would like to use the other features and is willing to test them, I'd be happy to help.

Install VRPN library steps:
1 - Clone the repository
```
git clone https://github.com/vrpn/vrpn.git
```

2 - Checkout to a stable version
```
cd vrpn &&\
git checkout v07.35
```

3 - Compile the code and install system-wide
```
mkdir build &&\
cd build &&\
cmake ../ &&\
make &&\
make install DESDIR=/usr
```