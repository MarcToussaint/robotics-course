===========================
 SimLab Exercise 4 - Grasp
===========================

Before new exercises, always update the repo::

  cd $HOME/git/robotics-course
  git pull
  git submodule update
  cd build
  make -j4

* This is the last formal exercise. After this you're free to follow
  on with your projects.
* In this exercise you are allowed to either shortcut perception (use
  the 'cheat perception' as described below and in the sample code),
  or -- better -- to use the perception pipeline you developed in the
  previous exercise. You can always use the cheat perception to debug
  your perception pipeline.




a) Grasp, lift & drop a hopping ball
====================================

In ``05-grasp`` you already find a minimal setting where a dropping
and hopping ball is grasped. The hopping is realized in simulation by
an *imp* -- which is a little callback code that can perturb the state
of the simulation after each simulation step. This helps simulating
stochastic effects and failures.

The current implementation uses the 

1. Use your OpenCV code instead of cheat perception to continuously
   track the sp here. Keep the your model object always tracking the
   real red sphere.
2. The current code uses plain IK with a pseudo inverse from exercise
   1 to generate motion (you can stack the Jacobians and residuals for
   multiple objectives, see the motion slides). Try to replace this by
   a KOMO method that solves, in each iteration, for the optimal final
   grasp pose, and then generate continuous motion by commanding a
   small constant velocity towards that final grasp pose. Close the
   gripper when the final grasp pose is reached.
3. The current code sometimes makes the arm occlude the red ball. Can
   you choose a better alignment to avoid ball occlusion?
4. Optimize things to be fast enough (but still smooth) to actually
   grasp the hopping ball reliably.
5. After the successful grasp, lift the gripper (e.g., using KOMO to compute a good final lift/drop pose for the ball).
6. Then call ``openGripper("R_gripper")`` to drop the ball again.
7. Ideally, repeat grasp-lift-drop robustly in an endless loop.


b) Throw the ball
=================

Do everything as above, but instead of just dropping the ball, throw
it (far) using the robot. How can you design a motion that has a
desired object velocity ad the point of ``openGripper``?


c) Bonus: ...and for a box
==========================

Repeating the above exercise for a box is rather involved, esp. if you do not cheat perception and have to retrieve the box orientation (long axis) using opencv. Therefore, for this exercise it is fine for you to use cheat perception.

1. Replace the red sphere by a sphere-swept box of size ``[.06, .06,
   .15, .01]`` (in both, the RealWorld and your model world).
2. When localizing the box, retrieve both, the center and the rotation
   matrix. Note that the third column in the rotation matrix is the
   unit vector which points into the direction where the box is 15cm
   long. Let A be this direction.
3. Design a box gripping final pose, where the gripper x-axis is
   orthogonal to A, and orthogonal to world-z. Does this suffice to
   design a good box grasp? (Cf. Exercise 2)


