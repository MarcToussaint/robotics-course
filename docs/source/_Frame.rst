=====
Frame
=====

A ``Frame`` is the elementary data structure within a
configuration. The core property of a frame is its 3D pose (position &
orientation). Each frame can have a parent frame, and in that case it
can also have degrees of freedom (dofs) to parameterize its relative
pose to the parent. A frame can also have a 3D shape (mesh)
associated. And finally it can have Inertia associated. In advanced
applications, a Configuration can also maintain a list of object
interactions and their dofs (e.g., contact forces).

Users get a handle to a frame only when they add it to a configuration
with ``C.addFrame``, or get it directly via ``C.getFrame``. With such
a handle, they can set frame properties, such as its name, parent,
(relative) position, (relative) orientation; its 3D shape as mesh,
primitive or point set, and color; its joint properties if it has
DOFs; its inertia; and its contact flag.

Basic properties: Name, parent, position, orientation
=====================================================

TODO: setName, setParent

.. automethod:: libry.Frame.setPose
.. automethod:: libry.Frame.setPosition
.. automethod:: libry.Frame.setQuaternion
.. automethod:: libry.Frame.setRelativePosition
.. automethod:: libry.Frame.setRelativeQuaternion

Getters:

..  libry.Frame.getPose
.. automethod:: libry.Frame.getPosition
.. automethod:: libry.Frame.getQuaternion
.. automethod:: libry.Frame.getRotationMatrix
.. automethod:: libry.Frame.getRelativePosition
.. automethod:: libry.Frame.getRelativeQuaternion

Shape properties
================

.. automethod:: libry.Frame.setShape
.. automethod:: libry.Frame.setPointCloud
..  libry.Frame.setConvexMesh
..  libry.Frame.setMesh
.. automethod:: libry.Frame.setMeshAsLines
.. automethod:: libry.Frame.setColor

Getters:

.. automethod:: libry.Frame.getMeshPoints
..  libry.Frame.getMeshCorePoints


Joint properties
================

.. automethod:: libry.Frame.setJoint

Inertial properties
===================

.. automethod:: libry.Frame.setMass

Collision properties
====================

.. automethod:: libry.Frame.setContact

