=============
Configuration
=============

Just as arrays (vectors, matrices) are the core data structure to code
numerics, a Configuration is the core data structure to code
robotics. A configuration represents objects in a 3D world and how
they are kinematically linked. When coding robotics, you create
configurations, edit them, compute numerical features from them (such
as the 3D position of an object, or the distance between two shapes),
and formulate optimization problems over configurations to solve
control or path finding problems.

A ``Configuration`` is a set of ``Frames``, each of which can have a
parent frame, can have degrees of freedom (dofs) to parameterize its
relative pose to the parent, can have a ``Shape``, and can have
``Inertia``. In advanced applications, a Configuration can also maintain
a list of object interactions and their dofs (e.g., contact forces).

``Features`` are the core concept that relate configurations to
numerics: Any numerical quantity you might want to compute for a
configuration is a ``Feature``. This could be basic things like the
position or orientation of a frame, or the distance between shapes,
or nearest points on two shapes. But also more involved things like
the total engergy in the configuration (if velocities are also
represented). Often features represent errors or residuals
of equation systems. For instance, you would often define a feature to
be the residual between an object 3D position and a desired 3D
position, rather than the absolute object position. Or a feature can
be the residual of equations for force propagation or the residual of
closed kinematic loops. In that way it is convenient to formulate
optimization problems that aim to bring all features to zero (or lower
equal to zero). Such optimization problems can solve control and path
problems for you. Features are the interface between a configuration
and numerics.

The relative transformations in configurations can have *degrees of
freedom* (dofs), thereby modelling kinematic joints. The vector of all
these dofs is also called joint state. A fundamental operation is to
"set the joint state" of a configuration, and then computing all
relative and absolute transformations of all frames, which is called
forward kinematics. The classical joint state is the main example for
dofs -- but in general we may want to introduce also other dofs,
namely dofs that represent the force interaction between objects, or
variables that indicate the time interval between two consecutive
configurations. The reason is that these dofs can then become the
decision variables in a mathematical program that solves for these
quantities. Therefore, the general view is that whatever we might want
to have as decision variable in a mathematical program should be dofs
in configurations.

C'tor
=====

.. automethod:: libry.Config.__init__

Adding frames to the configuration
===================================

.. automethod:: libry.Config.addFile
.. automethod:: libry.Config.addFrame
.. automethod:: libry.Config.delFrame
.. automethod:: libry.Config.getFrame

Copy, clear
===========

.. automethod:: libry.Config.copy
.. automethod:: libry.Config.clear

Joint state: get/set, and selecting active joints
=================================================

.. automethod:: libry.Config.setJointState
.. automethod:: libry.Config.getJointNames
.. automethod:: libry.Config.getJointDimension
.. automethod:: libry.Config.getJointState
		
.. automethod:: libry.Config.selectJoints
.. automethod:: libry.Config.selectJointsByTag

Frame state
===========

.. automethod:: libry.Config.setFrameState
.. automethod:: libry.Config.getFrameNames

Features: Anything you might want to compute from Configurations
================================================================

.. automethod:: libry.Config.feature
.. automethod:: libry.Config.evalFeature

Structural edits
================

.. automethod:: libry.Config.attach
.. automethod:: libry.Config.sortFrames
.. automethod:: libry.Config.edit

Collision computations
======================

.. automethod:: libry.Config.computeCollisions
.. automethod:: libry.Config.getCollisions


Factory functions to create other classes (KOMO, Simulation, viewer, etc)
=========================================================================

.. automethod:: libry.Config.komo_IK
.. automethod:: libry.Config.komo_CGO
.. automethod:: libry.Config.komo_path
.. libry.Config.bullet
.. libry.Config.physx
.. automethod:: libry.Config.simulation

