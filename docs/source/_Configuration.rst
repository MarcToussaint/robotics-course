.. _refConfiguration:

=============
Configuration
=============

Synopsis
===

A ``Configuration`` is the core data structure in ``rai`` to represent
a scene with objects and robots, similar to a scene graph in
graphics. Essentially it stores a set of (coordinate) ``Frames``, to
each of which we can associate a ``Shape``, ``Inertia``, interaction
forces, etc. Further, a configuration has ``DoFs``
(degrees-of-freedom), e.g. robot joint angles, which together form a
vector. This vector representation of a configuration allows to apply
optimization, control, and path finding methods on a
configuration. Finally, a configuration allows you to compute
``Features``, which is any numerical quantity you might want to
compute about a configuration. This could be basic things like the
position or orientation of a frame, or the distance between shapes, or
nearest points on two shapes, but also more involved things like the
error of motion under Newtonian dynamics. Features are the interface
between a configuration and optimization (or constraint solving)
algorithms.

A configuration can also represent a ``sequence of scenes'', i.e., a
time discretized path. In this case, the configuration stores
:math:`T` frames for a single objects, which all share the same shape
and describe the motion of the object. Although the semantics of such
a ``path configuration'' and normal configuration differ, in
``rai`` we use exactly the same data structure ``Configuration`` for both,
as all the methods and operations we want to perform on them are
essentially the same: We can parameterize a path configuration
(trajectory dofs), and compute features on a path configuration
(including velocity/acceleration/physics-based features).

A configuration is also an interface between various engines:
Rendering (OpenGL), Physics engines (bullet, PhysX), and file formats
(`rai`'s .g-file, URDF, collada).


Formally
===

In a rigid body case we would define a configuration :math:`C \in
\SE(3)^m` as :math:`m` coordinate frames. To each frame a shape and/or
inertia may be associated. We assume a configuration has dofs
:math:`x\in\mathbb{R}^n` and write :math:`C(x)`. Given a
configuration, we can compute features :math:`\phi(C(x)) \in
\mathbb{R}^d` and Jacobians :math:`\nabla_x \phi(x) \in
\mathbb{R}^{d\times n}` of features. This is the role of the
`Configuration` data structure.

Example Uses
===

Scene loading, editing, generation
---

* Loading URDF -- use `ry.tool.urdf2rai()`; often requires manual fixes. See details here.
* Editing .g-files -- use `ry.tool.kinEdit()`! See details here.
* editing and generation by code
  
Example: Load 2 robots into a file, attach the root of one robot to the endeff of the other, add and attach an own object, animate it, store it as urdf and collada.


Dofs, set/get state, seleting active dofs
---

Computing Features
---

Basic Inverse Kinematics
---

The frame state
---
While the dof state is :math:`x\in\mathbb{R}^n`, the frame state is :math:`C(x) \in SE(3)^m`. That is, the frame state is the pose (position+quaternion) of *all* frames of the configuration. It is set and returned as a (m,7)-matrix.

Note the quirks in setting a frame state when we also have joints at the same time! Setting the frame state of all frames overwrites also all relative transformations, ignoring that a joint associated to the relative transformation might actually only parameterize a limited transformations (e.g. a hinge rotation). So setFrameState may easily set relative transforms to be inconsistent with joint constraints. A following setJointState would simply overwrite these relative transformations again. More interestingly, a getJointState following a setFrameState extracts the dofs along the joint dimensions from the the freely set relative transformation, and a following setJointState would result in a joint-consistent transformation as close as possible to the freely set relative transformation.



Computing Collisions
---
Many features require computation of potential collisions between all shapes. `rai` uses collision engines (FCL for broadphase, and lower-level ccd/GJK calls for precise collision information) to this end. A setState does *not* automatically trigger calling the collision engines. Some features such as `F_accumulatedCollsions` automatically ensure that these collision engines were called. However, the user can also explicitly ensure updated collision information by calling `ensureProxies`, after which collisions can be reported. This can be used, e.g., to implement an interface for path finding algorithms (e.g., OMPL).

Parents, Sub-Frames, & Links
---
Frames typically have parent frames. If a joint is associated to this parent relation, then this typically indicates that the relative transformation to the parent has DOFs (special case: A joint can also be rigid). If no joint is associated to the parent relation, then the child is really a sub-frame and part of the same `link`.

A `link frame` is one that has no parent or a joint to its parent. For a link frame we can (recursively) collect all sub-frames, which together describe a `link` (e.g. an object with multiple sub-shapes or inertias with different relative transformations to the link frame).


Activating/Filtering Collisions
---
Shapes have a ``cont`` property (TODO: change to int collide). If collide=0, collision evaluation is deactivated and it collides with no other shape; if collide=1 collision evaluation is activated; collisions of shapes within the same link are generally filtered out; if collide<0, then collision evaluation is activated *but* collisions are filtered out not only to other shapes in the same link, but also other shapes in the parent-link of order '-collide', e.g., the two *links* tree-upward for collide=-2.

  

  is only roughly supp

the pose (position orientation) of $m$ objects, equally including environment or (multi-) robot parts. A shape or inertia may be associated to each obj


Just as arrays (vectors, matrices) are the core data structure to code
numerics, a ``Configuration`` is the core data structure to code
robotics. A configuration represents objects in a 3D world and how
they are kinematically linked. When coding robotics, you create
configurations, edit them, compute numerical features from them (such
as the 3D position of an object, or the distance between two shapes),
and formulate optimization problems over configurations to solve
control or path finding problems.


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
freedom* (dofs), thereby modelling kinematic ``Joints``. The vector of all
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

A configuration is often initialized by reading it from a file. See
:ref:`refConfigurationFile` for details on these files.

.. automethod:: libry.Config.addFile

Alternatively, a configuration can also be build up within the code by
adding more and more frames. See :ref:`refFrame` on how to specify the
properties of frames.

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


Functions to create other classes (KOMO, Simulation, viewer, etc)
=========================================================================

.. automethod:: libry.Config.komo_IK
.. automethod:: libry.Config.komo_CGO
.. automethod:: libry.Config.komo_path
.. libry.Config.bullet
.. libry.Config.physx
.. automethod:: libry.Config.simulation

