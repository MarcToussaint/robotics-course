===================
 SimLab Exercise 1
===================


Code Setup
==========

Install and test
https://github.com/MarcToussaint/robotics-course. Follow the
instructions in the README (`Setup for Lab Course
in Simulation
<https://github.com/MarcToussaint/robotics-course#setup-for-robotics-practical-in-simulation>`_)
   
The specific goals are

* to compile the repository
* to run your first tests in ``course3-Simulation/01-`` and ``02-``
* setup your coding environment in C++ (eg. qtcreator or VS Code) or python (pycharm, jupyter notebooks, VS Code)
* Have a concept how you use git! (See below.)


Basic motion
============

Revive what you've learned in the robotics course about computing
poses and generating motion. In particular, think already about
grasping. What would be a way to make the robot grasp a sphere,
cylinder, box, or general shape?

Specific goals to get started:

* In the examples ``course3../02-`` add a virtual object to the scene. Move the left arm towards that object.
* Be able to show, in the class, examples of more complex motion. Anything beyond the given example.
* Read the ``tutorials/2-features.ipynb`` to get an impression about what alternative features one can use to design motion.
* Think about how grasping could be realized (aligning the gripper, approach of the gripper)
* To be discussed in class: How open/close gripper is realized in simulation


Git Setup
=========

You definitely should use a separate git repo for your own code. You
will also have to pull new versions of robotics-course in the
future. I recomment to create your own repo on github (or university
gitlab), and clone it next to the robotics-course repo in
``$HOME/git``. There are two alternatives:

* Your repo only contains your own py or cpp code, with Makefiles pointing to the robotics-course repo (recommended for simplicity)
* Your repo has the robotics-course as submodule. This is more self-contained, but requires you to understand more about git

Specific goals if you're yet unfamiliar with git:

* Read the git tutorial \url{https://try.github.io/}
* Create your own group git repo somewhere
* Know how to create different branches, e.g.\ for parallel development of different group members
* Each group member should have made at least one commit, e.g.\ with a trivial hello text file
* Copy & Paste example code from ``course3-../02-`` to your own repo and get it to compile there (C++: Have a look at the simple compile commands when you build the examples, or copy my Makefile and modify the BASE to point to robotics-course/rai.) (In python, set the sys path appropriately to robotics-course/build.)
