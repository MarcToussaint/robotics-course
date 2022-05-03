# MLR robotics course & practical robotics course

This repo is based on RAI code, including its python bindings. See https://github.com/MarcToussaint/rai for a README of the RAI code.

## Table of Contents
- [Quick Start](#quick-start)
  - [Documentation](#documentation)
  - [Setup for Robotics Lab Course in Simulation](#setup-for-robotics-lab-course-in-simulation)
  - [Setup for Robotics Lecture Exercises](#setup-for-robotics-lecture-exercises)
  - [Setup for the Robotics Lab Course with the real Baxter Robot Baxter](#setup-for-the-robotics-lab-course-with-the-real-baxter-robot-baxter)
- [Further Documentation & Installation Pointers](#further-documentation--installation-pointers)
  - [Installation](#installation)
  - [rai code](#rai-code)
  - [rai examples](#rai-examples)
  - [Tutorials](#tutorials)
  - [More details on handling baxter](#more-details-on-handling-baxter)
  - [Internals](#internals)


## Quick Start

The repo is now used for three lecture formats: the robotics lab
course in simulation, the robotics lab course in real, and the
robotics lectures. Please follow the respective sections.

### Documentation

* [Course material and some documentation of the code base and python bindings](https://marctoussaint.github.io/robotics-course/)


### Setup for Robotics Lab Course in Simulation

This assumes a standard Ubuntu 18.04 or 20.04 machine.

* The following assumes $HOME/git as your git path, and $HOME/opt
to install 3rd-party libs -- please stick to this (no system-wide installs)

* If you'll use python:
```
sudo apt-get install python3 python3-dev python3-numpy python3-pip python3-distutils
echo 'export PATH="${PATH}:$HOME/.local/bin"' >> ~/.bashrc   #add this to your .bashrc, if not done already
pip3 install --user jupyter nbconvert matplotlib pybind11 opencv-python
```

* Clone and compile our robotics-course code:
```
mkdir -p $HOME/git
cd $HOME/git
git clone --recursive https://github.com/MarcToussaint/robotics-course.git

cd robotics-course
make -j1 installUbuntuAll  # calls sudo apt-get install; you can always interrupt
# If this fails, please try `make -j1 printUbuntuAll` to print all packages and install manually
# if you are going to use opencv, run also 
# sudo apt install libopencv-dev

mkdir build
cd build
cmake ..

make -j $(command nproc)
```

* If you use python, run tests:
```
jupyter-notebook tutorials/1-basics.ipynb
jupyter-notebook course3-Simulation
```

* If you use C++, compile and run the tests:
```
cd course3-Simulation/01-test
make
./x.exe
#and the same for all other course3-Simulation/... tests
```

* Alternative non-cmake build system (not recommended, but allows to configure config.mk):
```
cd $HOME/git/robotics-course
rm -Rf build
make -j $(command nproc)
ln -s rai/lib build
```

* When pulling updates for the repo, don't forget to also update the submodules:
```
git pull
git submodule update
```

* We tested this (sometimes) in docker. See
  [here](https://github.com/MarcToussaint/rai-maintenance/tree/master/docker/)
  for a collection of docker setups. In mini20 the above install was
  tested. full18 includes a pre-compiled PhysX.


* When enabling Physx (as alternative to bullet), first install PhysX
  from source as described
  [here](https://github.com/MarcToussaint/rai-maintenance/blob/master/help/localSourceInstalls.md#PhysX),
  then add Physx lib path to LD_LIBRARY_PATH
```
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/opt/physx3.4/lib    # add path (temporary)
echo 'export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$HOME/opt/physx3.4/lib' >> ~/.bashrc     # or add permenantly in bashrc
```


### Setup for Robotics Lecture Exercises

This assumes a standard Ubuntu 18.04 machine.

In this course, check that in 'config.mk' we have (disabling lots of stuff)
```
ROS=0
OPENCV=0
PHYSX=0
BULLET=0
CERES = 0
NLOPT = 0
```

```
git clone --recursive https://github.com/MarcToussaint/robotics-course.git
cd robotics-course

make -j1 installUbuntuAll  # calls sudo apt-get install; you can always interrupt
# If this fails, please try `make -j1 printUbuntuAll` to print all packages and install manually

make -j $(command nproc)   # builds libs and tests
ln -s rai/lib build
```

To test the python notebooks:
```
# export PATH="${PATH}:$HOME/.local/bin"   #add this to your .bashrc, if not done already
pip3 install --user jupyter nbconvert matplotlib
jupyter-notebook tutorials/1-basics.ipynb 
```
After loading the pr2 and the kitchen (running first 3 cells in the notebook), the simulator window should look similar to:
![Alt text](screenshot.png?raw=true "Title")




### Setup for the Robotics Lab Course with the real Baxter Robot Baxter

* Install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
* Source and install
```
pip install wstools catkin_pkg --user
source /opt/ros/kinetic/setup.bash
```
* The following assumes all git repos are cloned into $HOME/git
* clone
```
mkdir -p ~/git
cd ~/git
git clone --recursive https://github.com/MarcToussaint/robotics-course.git
cd robotics-course
```
* change `ROS = 0` to `#ROS = 0` in `config.mk` 
* install also baxter sources using
```
cd course1-Lectures/external
./installBaxterSources.sh
```
* compile
```
make -j1 installUbuntuAll  # calls sudo apt-get install; you can always interrupt
make -j4                   # builds libs and tests
```
* if using c++, install `qtcreator` as described [here](https://github.com/MarcToussaint/rai-maintenance/blob/master/help/qtcreator.md)
* when in the lab, connect to the wifi mlr-robolab (password: mlr-robolab)
* call `source bin/baxterwlansetup.sh` from ~/git/robotics-course
* source ROS and the baxter sources
```
source /opt/ros/kinetic/setup.bash
source external/devel/setup.bash
```
* IF YOU'RE THE ONLY ONE USING BAXTER, turn on baxter and call `bin/baxterStart.sh`
* Try `rostopic list`
* Try the cpp example
```
cd course2-Baxter/01-baxterMini
make
./x.exe -useRos 1
```
* Try the python example
```
cd course2-Baxter/01-baxterMini
jupyter-notebook motion.ipynb 
```
* Before turning off baxter, run `rosrun baxter_tools tuck_arms.py -t`


# Further Documentation & Installation Pointers

## Installation

* [ROS kinectic](http://wiki.ros.org/kinetic/Installation/Ubuntu) (for Ubuntu 16.04) or [ROS melodic](http://wiki.ros.org/melodic/Installation/Ubuntu) (for Ubuntu 18.04)
* [OpenCV](https://github.com/MarcToussaint/rai-maintenance/blob/master/help/localSourceInstalls.md#OpenCV) (from source)
* [PhysX](https://github.com/MarcToussaint/rai-maintenance/blob/master/help/localSourceInstalls.md#PhysX) (from source)
* [Bullet](https://github.com/MarcToussaint/rai-maintenance/blob/master/help/localSourceInstalls.md#Bullet) (from source)
* [qtcreator](https://github.com/MarcToussaint/rai-maintenance/blob/master/help/qtcreator.md) (from source or ubuntu, setting up projects, pretty printers, source parsing)
* Python3:
```
sudo apt-get install python3 python3-dev python3-numpy python3-pip python3-distutils
sudo update-alternatives --install /usr/bin/python python /usr/bin/python3.6 1
```

## rai code

* [rai::Array and arr](https://github.com/MarcToussaint/rai-maintenance/blob/master/help/arr.md) (tensors, arrays, std::vector)
* [Features and Objectives for KOMO](https://github.com/MarcToussaint/rai-python/blob/master/docs/2-features.ipynb)
* [Graph and `.g` files](https://github.com/MarcToussaint/rai-maintenance/blob/master/help/graph.md) (Python dictionaries, any-type container, file format, logic)
* [Editing robot model/configuration files](https://github.com/MarcToussaint/rai-maintenance/blob/master/help/kinEdit.md)  (URDF, transformations, frame tree)
* [docker](https://github.com/MarcToussaint/rai-maintenance/tree/master/docker) (testing rai within docker, also Ubuntu 18.04)

## rai examples

* [Python examples](https://github.com/MarcToussaint/rai-python/tree/master/docs)
* [Python robotics exercises](https://github.com/MarcToussaint/robotics-course/tree/master/py)
* [Cpp robotics exercises](https://github.com/MarcToussaint/robotics-course/tree/master/cpp)

## Tutorials

1. [Basics:](tutorials/1-basics.ipynb) Configurations, Features & Jacobians
1. [Features:](tutorials/2-features.ipynb) Learn about the language to define and query features and their Jacobians. Including querying collision features (whether and which objects are in collision).




## More details on handling baxter

### Booting

On the back of the robot near the pedestal base, there is a power button. Push it and wait for the machine to finish booting.

### Communicating with Baxter

Connect to the lab mlr-robolab WLAN (password: mlr-robolab)

Baxter runs with ROS, and you'll need to set your environment
variables to enable ROS communication. The easiest way to do this is
to connect to run one of following scripts in Terminal from the mlr
folder:

```
source bin/baxterwlansetup.sh
```

### Start-up

Call the start-up script, which enables baxter, untucks the arms,
turns off the ultrasonics (they click very loudly in any videos), and
calibrates the grippers.

```
bin/baxterStart.sh
```
Kill /end_effector_publisher node which corrupts the /robot/joint_states.
This only has to be run once when the robot is turned on.

```
rosnode kill /end_effector_publisher
```

### Using vacuum gripper

Switch on the air pump. Pull and slowly turn the black valve to make the air pressure around 60-100psi (preferably **70 psi**, the pressure will increase slowly so turn the valve slowly as it increases) Once the pressure is set the pump will automatically keep its pressure. Turn off air pump when finished using. 

The gripper can be accessed same as the electric gripper.


### Accessing camera

* To launch the ASUS camera, install openni2. Change kinetic to a different ROS version in the command if you're not using Kinetic.
```
sudo apt-get install ros-kinetic-openni2-launch
```
Plug in the camera USB and run the following command.
```
roslaunch openni2_launch openni2.launch depth_registration:="true" hw_registered_processing:="true" color_depth_synchronization:="true" auto_exposure:="false" auto_white_balance:="false"
```

* To launch the Kinect camera, install freenect. Change kinetic to a different ROS version in the command if you're not using Kinetic.
```
sudo apt-get install ros-kinetic-freenect-stack
```
Plug in the camera USB and run the following command.
```
roslaunch freenect_launch freenect.launch camera:="kinect"
```
### Shutdown

Always tuck the arms before shutting down, to keep the spring wear to a minimum.
```
bin/baxterTuck.sh
```
or
```
rosrun baxter_tools tuck_arms.py -t
```

Then press the power button once to turn the robot off.

Alternatively, you can ssh in to the robot (password: rethink) and run:
```
ssh ruser@thecount.local
sudo shutdown -h now
```


### Troubles

* One some machines, OpenGL with the glfw seems broke. You'll have to change back to an older version which uses freeglut. For this, in `rai/Gui/Makefile` switch the 0/1 for `FREEGLUT` and `GLFW`

* Beware ros node names!! (Maybe it is good if everybody uses the same rosNodeName? That way they block each other? Behavior undefined!)


## Internals

Within the submodules, to set ssh access, call:
```
git remote set-url origin git@github.com:MarcToussaint/rai.git
git remote set-url origin git@github.com:MarcToussaint/rai-robotModels.git
```
