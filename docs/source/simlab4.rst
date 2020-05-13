===========================
 SimLab Exercise 4 - Grasp
===========================

Before new exercises, always update the repo::

  cd $HOME/git/robotics-course
  git pull
  git submodule update
  cd build
  make -j4


  
CODE CHANGES:

* Added `tutorials/05-howto.ipynb`, which demonstrates how to kinematically attach objects (fake grasp), also for attaching the camera to the endeff, and understand broadphase collision checking (enabling/disabling collisions)
* I added documentation -- see the 'Core Data Structure' in the menu.
* SHIFT-mouse1 allows to move around the view (previous button2 didn't seem to work for some of you); as before, mouse3 sets focus



The goal of this session is to combine

* Use OpenCV to localize/track an object
* Grasp and lift the object

The key difficulty here is in translating the perception into a
representation (e.g. primitive shape) in your model that allows for
grasping.

In ``05-grasp`` you find tips to simplify the RealWorld
scene, allowing you to give arbitrary colors and shapes to the objects
dropping. Thereby you can start first working on grasping simple
(small) blocks, and then gradually tackle harder problems.


Exercise 1:
===========

* Drop a single small (e.g. 3cm radius) sphere from the scene
* Start developing your own perception pipeline that you will also use
  in the future: Here is just a suggestion:
  
  * Segment the object from the background (using color, or depth,
    with or without background substraction), resulting in a binary
    mask
  * collect all image points that belong to the object into a
    n-times-3 matrix with rows (x-pixel-coordinate,
    y-pixel-coordinate, depth-meters) -- this is the point cloud in
    image coordinates
  * Transform that point could into meter coordinates (still relative
    to the camera frame) -- resulting in a n-times-3 matrix
  * Display this point could by attaching it as "shape" to the
    cameraFrame (using cameraFrame.setPointCloud, see example
    04-opencv)

* Now you have the point cloud in the model - but that's still not
  good for grasping. Since it is a ball, we can reduce it to just
  the mean point:

  * Compute the mean of the point cloud
  * Ensure you have earlier created some frame/object in your model
    that now becomes the representation of the percept: set the
    position of the model object to the point cloud center

* Generate a motion to grasp the ball from top down
* Close the gripper (see example in grasp test in ``01-test``)
* Lift it!
* Open the gripper again


Exercise 2:
===========

Make the above as robust and reactive as possible.

* Regrasp the ball every time you drop it
* Does throwing work? (Can you drop it with significant velocity?)
* I'm implementing an "adversarial imp", which can perturb the RealWorld randomly. With this imp, the ball will sometimes jump on the table. Still try to grasp it reactively

Exercise 3:
===========

* Change the shape type from sphere to ssBox with size [.05 .05 .2 .01]
* How can you find the direction with which to align the gripper?
