=================================
 SimLab Exercise 5 - Grasp Again
=================================

Before new exercises, always update the repo::

  cd $HOME/git/robotics-course
  git pull
  git submodule update
  cd build
  make -j4


Memos from our discussion:

* This is the last formal exercise. After this you're free to follow
  on with your projects. (Thanks for your creative ideas!)
* In that last exercise I'd like to push you once again to realize a
  grasp in simulation, both, for a sphere and a box -- using the
  simulation interface of open/closeGripper only. It is a constraint
  for your final project to involve at least some kine of object
  manipulation (grasping or pushing).
* I will add methods to shortcut perception to the Simulation
  interface, something like ``getGroundTruthObjectPose(objectName)`` and
  shape. Esp. for sequential manipulation of multiple objects (tower
  stacking challenge) it might make sense to using the shortcuts to
  get deeper into sequential manipulation problems. If you continue
  the hard way of including a perception pipeline in the system, you
  can use the shortcut to debug.


In ``05-grasp`` you find tips to simplify the RealWorld
scene, allowing you to give arbitrary colors and shapes to the objects
dropping. Thereby you can start first working on grasping simple
(small) blocks, and then gradually tackle harder problems.


Exercise 1:
===========

To generate the motions, build on your solutions of exercise 2 (using
KOMO), or exercise 1 (using plain IK with a pseudo inverse (you can
stack the Jacobians and residuals for multiple objectives, see the
motion slides)).

* Drop a single small (e.g. 3cm radius) sphere from the scene
* Continuously track the sphere using OpenCV or using the shortcut method
* First start a motion that aims to not occlude the camera: E.g., let the ``R_gripper`` x-axis be equal to world y-axis; and let the gripper-z be equal to (1/sqrt(2),0,1/sqrt(2)). Note: that fully specifies the orientation (you could also use a full quaternion task) to grasping from the right.
* Keeping this orientation, move the gripper towards the ball to align the position.
* Have a continuous control loop running to stabalize the position alignment
* If the positions are aligned well, call ``closeGripper("R_gripper")`` once, but keep the position control loop running
* Continuously test for the ``getGripperIsGrasping("R_gripper")`` boolean. If it turns true, exit this control loop -- the ball is attached within simulation.

After this, lift and drop:

* Use IK again to impose a upward velocity on the ``R_gripper``, to lift it together with the ball
* After some time, stop lifting and call ``openGripper("R_gripper")``

The ball should drop in simulation. Move the gripper out of occluding the view, and repeat the whole pipeline indefinitely (if the ball does not roll away too far).



Exercise 2:
===========

* Repeat the above for a box
* Drop a single red box of size ``[.06, .06, .15, .01]`` into the scene

This time you will have to grasp the block from top -- so you cannot avoid occlusion. Therefore follow a slightly different strategy:

* Track the box using perception or the shortcut -- retrieve both, the center and the rotation matrix. Note that the third column in the rotation matrix is the unit vector which points into the direction where the box is 15cm long. Let A be this direction.

* Position your arm 10cm above the object, with the grippers z-axis equal to world z-axis; and the grippers y-axis equal to A.

* Move the gripper down to align with the box position, keeping the orientation tasks

* Close the gripper, lift, and open again.

* Move the gripper out of occluding the view and repeat the process indefinitely.
