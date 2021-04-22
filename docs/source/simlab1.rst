=================================
 SimLab Exercise 1 - Basic Setup
=================================


a) Code Setup
=============

Install and test
https://github.com/MarcToussaint/robotics-course. Follow the
instructions in the README (`Setup for Lab Course
in Simulation
<https://github.com/MarcToussaint/robotics-course#setup-for-robotics-lab-course-in-simulation>`_)
   
Specific delivarables:

* You have compiled the repository
* You can run the tests in ``course3-Simulation/01-`` and ``02-``
* You have setup your coding environment in C++ (eg. qtcreator or VS Code) or python (pycharm, jupyter notebooks, VS Code) - you need code browsing & debugging support!



b) Git Setup
============

You definitely should use a separate git repo for your own code. You
will also have to pull new versions of robotics-course in the
future. Please create your own repo on github or the university
gitlab. There are two alternatives to organize your own repo:

* You fork the robotics-course repo and work directly in the fork (with your exercise solutions and own code in your own new folder usr/)
* You create your own fresh repository next to the robotics-course repo, which only includes subfolders with your code/solutions. You copy the 01-test/Makefiles to your working folder but simply change the first line to ``BASE = ${HOME}/git/robotics-course/rai``
* You create your own fresh repository and set it up analogous to the robotics-course repo. This is not too difficult, as the important code is all in submodules. You'll have to add the following submodules::
    
   git submodule add https://github.com/pybind/pybind11.git
   git submodule add https://github.com/MarcToussaint/rai.git
   git submodule add https://github.com/MarcToussaint/rai-robotModels.git
   cd rai; git checkout robotics-course; cd.. #to get the submodule branch we work with

Then add also the CMakeLists.txt, and that should be all you need.

Specific deliverables:

* You have your git working repo setup
* You give us read access to your repo via the ISIS page

If you're yet unfamiliar with git,

* Read the git tutorial https://try.github.io/
* Create your own group git repo somewhere
* Know how to create different branches, e.g.\ for parallel development of different group members
* Each group member should have made at least one commit, e.g.\ with a trivial hello text file



c) Rough familiarity with Simulation and Configuration
======================================================

The ``02-basics`` example is your starting point for own code. Copy
this to your own working repo.

The :ref:`refConfiguration` is THE core data structure to work with. Read its docs.

The :ref:`refSimulation` is really very rough (there is hardly code behind
it). It simply attaches to a Configuration that is meant to represent
the real world, and performs some simple operations such as moving the
robot in that configuration, calling an external physics library to
animate objects, rendering an RGB and depth image from the scene with
plain OpenGL depending on the camera frame. Read the docs of _Simulation.

Specific deliverables:

* Be able to walk through the Simulation.h (C++ header) and explain roughly each method.
