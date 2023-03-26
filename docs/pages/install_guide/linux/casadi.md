# Install CasADi C++ Library

## Installing IPOPT

1. Start off by grabbing as many dependencies as possible from repositories.
```
sudo apt-get install gcc g++ gfortran git cmake liblapack-dev pkg-config --install-recommends
```

2. To compile the Python interface, you also need SWIG and a decent Python installation:
```
sudo apt-get install swig ipython python-dev python-numpy python-scipy python-matplotlib --install-recommends
```

3. On Ubuntu (or Debian) Linux and Mac OS X you can obtain IPOPT from a package manager:
```
sudo apt-get install coinor-libipopt-dev
```

## Installation CasADi

1. First, you have to get CasADi from http://www.casadi.org/ or check out the git repository from GitHub.
```
git clone https://github.com/casadi/casadi.git -b ge6 casadi
```

2. Set the environment variable CMAKE_PREFIX_PATH to inform CMake where the dependecies are located. For example if Sundials headers and libraries are installed under $HOME/local/, then type
```
export CMAKE_PREFIX_PATH=$HOME/local/
```

3. Go to directory of the source tree, create a directory called e.g. ./build where all compilation-related files will be created.
```
cd casadi; mkdir build; cd build
```

4. Generate the Makefile by typing
```
cmake -DWITH_IPOPT=ON -DCMAKE_INSTALL_PREFIX:PATH=/usr ..
```

5. Check the output of CMake if the dependencies are correctly located. The interfaces without underlying libraries and headers won't be compiled. Adjust CMake variables (e.g. CMAKE_INSTALL_PREFIX if you want to install headers and libraries) by typing
```
ccmake ..
```

6. Now compile the C++ libraries.
```
make
```

7. And install headers and libraries
```
make install
```

8. If you wish to use the python interface you can compile it with the python target.
```
make python
```


