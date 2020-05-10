==========
Simulation
==========

The ``Simulation`` class attaches a physics engine to a configuration,
allows to step physics on that configuration and to control joints by
position, velocity, or accelerations, grab simulated RGB and depth
images, and add "adverserial imps" (that is, callback methods that
perturb the controls, configuration, or sensor signals in some way).

C'tor
=====

The c'tor attaches a physics engine to the given configuration and
initializes it. The current choices for ``engine`` are PhysX, Bullet,
and just kinematic (no physics engine). ``verbose=1`` will open a
display.

.. automethod:: libry.Simulation.__init__

Stepping physics
================

The core methods steps a physics engine. This distinguishes between
dynamic objects, which are governed by the laws of physics of the
physics engine, and kinematic objects, which typically are the robot
links. The method 1) applies the joint controls (positions,
velocities, or accelerations) to the associated configuration, 2)
pushes the kinematic objects (esp.\ the robot configuration) into the
physics engine, 3) steps the actual physics engine, 4) reads out all
frames of all objects and updates the associated configuration with
these. In addition, between each of these steps, an imp can introduce
perturbations.

``u_mode`` can be ``none`` for not sending control signals (in which case ``u_control`` can be ``[]``). Otherwise ``u_control`` needs to contain joint positions, velocities or accelerations for all DOFs.

.. automethod:: libry.Simulation.step

As the robot is explicitly controlled, the joint state should be clear
to the user. Still, for completeness two methods return the current
joint positions and velocities:

.. automethod:: libry.Simulation.get_q
.. automethod:: libry.Simulation.get_qDot

		
Opening & closing the gripper, and gripping state
=================================================

Many real grippers are controlled differently to standard robot
joints: They receive a signal to close and do so until a force limit
is reached. Similarly for opening. Therefore also this simulation has
separate methods to command grippers and also read out their state.

The open/close methods need the name of the gripper. The
speed/width/force are not yet implemented.

.. automethod:: libry.Simulation.get_q
.. automethod:: libry.Simulation.get_qDot

The gripper width can always be queried. And typical grippers also
tell you if they have an object in hand:

.. automethod:: libry.Simulation.getGripperWidth
.. automethod:: libry.Simulation.getGripperIsGrasping

Getting simulated RGB and depth images
======================================

Using OpenGL it is straight-forward to grab an RGB and depth image
from a camera. The Simulation allows you to specify a camera by
referring to a camera frame of the configuration, which should have
(focalLength, width, height, zRange) as attributes (defined in the
configuration description file \*.g). In C++, the cameraview access
exposes more ways to add and define sensors. The following methods
grabs RGB and depth from opengl, and properly transforms depth values
to have values in meters. Optionally, imps can post-process these
images to add noise and systematic errors.

.. automethod:: libry.Simulation.getImageAndDepth
.. automethod:: libry.Simulation.addSensor

Simulation states: restarting the simulation in previous states
===============================================================

It is often disirable to restart a physical simulation in an exact
same state that has been visited before. This is often missing in
standard physics engines. Infact, also the following methods are
slighly approximate, as they cannot exactly know and store some hidden
states of the engines' internal numerical solvers. What they store and
re-instantiate (``getState`` and ``restoreState``) is the exact poses
and velocities of all frames of the scene. (TODO: Also discrete facts, such as which grippers hold objects, must be stored.) The ``setState`` method
allows you to directly set arbitrary pose and velocity states.

.. automethod:: libry.Simulation.getState
.. automethod:: libry.Simulation.restoreState
.. automethod:: libry.Simulation.setState

Helper: depthData2pointCloud
============================

This should acutally be declared somewhere else. In C++ it is a global
method within the Perception code. A helper to convert a depth image
(already in meters) to a set of 3D points (a (W*H)x3-matrix). The
argument ``fxypxy`` need to be four numbers: the focal length (in
pixel/meters units!) in x and y direction, and the image center (in
pixel units) in x and y direction.

.. automethod:: libry.Simulation.depthData2pointCloud
