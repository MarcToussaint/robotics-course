.. _refConfigurationFile:

===========================================
 .g-files to describe robot configurations
===========================================

We use .g-files to represent robot/world configurations. .g-files
describe a general graph data structure as explained in
:ref:`refGraph`. But for robot configurations it is rather simple:
Everly node describes a frame, and is described by three things::

  <frame-name> ( <parent> ) { <attributes> }

where ``<parent>`` needs to be a previously defined frame, or omitted, if
the frame is a root frame. The attributes defined properties of the
frame, such as its pose, shape, joint properties, etc.

Here is an example taken from the ``test/Kin/kin``::
  
  stem { X:<t(0 0 .5)>, shape:capsule, mass:1, size:[1 .05] }
  
  joint1_pre (stem) { Q:<t(0 0 .5) d(90 1 0 0)> }
  joint1 (arm1) { joint:hingeX, Q:<d(-30 1 0 0)> }
  arm1 (joint1) { Q:<t(0 0 .15)>, shape:capsule, mass:1, size:[.3 .05] }
  
  arm2 { shape:capsule, mass:1, size:[.3 .05] }
  eff { shape:capsule, mass:1, size:[.3 .05] }
  
  joint2 (arm1 arm2) { joint:hingeX, A:<t(0 0 .15) d(0 0 0 1)>, Q:<d(-10 1 0 0)>, B:<t(0 0 .15) > }
  joint3 (arm2 eff ) { joint:hingeX, A:<t(0 0 .15) d(0 0 0 1)>, Q:<d(-10 1 0 0)>, B:<t(0 0 .15) > }
  
  target { X:<t(.0 .2 1.7)>, shape:sphere, mass:.001, size:[0 0 0 .02], color:[0 0 0] }

The first line defines a frame ``stem``, which has absolute pose
``<t(0 0 .5)>`` (pose specifications are described below). It also has
a shape attached, namely a capsule of length 1 and radius .05. And it
has inertial mass attached, namely with mass 1.

The 2nd to 4th line form a block of 3 new frames: the ``joint1_pre``
frame is child of step, which fixed relative transformation ``<t(0 0
.5) d(90 1 0 0)>`` (.5 meter up, 90 degress rotation about x). The
``joint1`` frame is a child of ``joint1_pre``. But this frame is
special! It is a joint, which means that its relative transformation
to the parent is not fixed, but varies with joint dofs. Here, it is 1
joint dofs describing a hinge joint about the parent's x-axis. This
joint is here initialized to non-zero, namely to a relative transform
``<d(-30 1 0 0)>``. The ``arm1`` frame is then a child of ``joint1``,
with a relative transform ``<t(0 0 .15)>``, a capsule shape attached,
and mass.

This is a typical example for a chain of frames from one link, via a
joint, to the next. All robot configurations are just trees; and the
configuration file simply defines frames one-by-one, where each frame
may have 1 parent frame.

The next two lines define two more frames ``arm2`` and ``eff`` mass
and capsule shapes; but they're yet all located at zero absolute
pose. The following two lines are actually a short hand notation to
introduce a joint frames between arm1 and arm2 (arm2 and arm3) in
retrospect. The ``joint2`` declaration implicitly first defines a
``joint2_pre`` child of arm1 with fixed relative transformation A;
then the ``joint2`` chile of joint2_pre with hinge joint and initial
transformation Q; then attaches the arm2 frame as its child with fixed
relative transformation B. So this is a typical short hand to specify
a joint (more similarly to how its done in URDF). But the generated
underlying data structure is just a tree of frames.


Editing using ``kinEdit``
=========================

Whenever working with .g-files, you should try to display them using
the ``$RAI/bin/kinEdit`` command line tool. CMake automatically
compiles it; otherwise call 'make bin' in $RAI to compile this. Then
you can call ``kinEdit someFile.g`` on any model file. (In
python, the equivalence is to reload the configuration from file
repeatedly.)  Whenever ``kinEdit`` reads a file, it also outputs a
file ``z.g`` and ``z.urdf`` of what it read. Sometimes it is useful to
look into this. It can also be used to clean and prune kinematic
structures.


But more than that, you can keep the display open when editing the
file in any text editor. Whenever you save the file the display will
notice it, reload the file, and display the updated model. This allows
some degree of continuous editing. You might sometimes have to hit
enter in the window to enforce reloading. The little tool tries to
catch and report on syntax errors and be robust, but it crashes on
some syntax errors and then needs to be restarted manually.


Import from URDF
================

You can convert URDF files to .g-files using the ``rai/bin/urdf2rai.py`` script. However, the overall conversion is only partially automatic.  The
resulting g-file encodes the full kinematic structure, but the mesh files usually require manual fiddling. First, in the
g-file, you have to change the path to their location in the file
system (removing the 'package' part). Potentially that's all you
need. However, the rai code calls various collision libraries that need clean
and correct (orientation, holes, etc) mesh files. Those that come with
URDF files are typically not clean and correct. I typically use
meshlab (the command line tool) to automatically clean and compress
meshes into ply files.

The best guide for the whole conversion pipeline is hubo/HOWTO.sh in
the rai-robotModels repository, which also describes mesh cleaning
scripts.  We also managed to import a [full
kitchen](https://github.com/MarcToussaint/rai-robotModels/tree/master/bremenKitchen)
from unreal, where we first exported the description to JSON.  There
are also some working examples to import `gltf`.

Finally, the collada file format can represent trees of frames and
objects, which can be loaded. This can be augmented with just a little
extra information on joints to make this a properly articulated robot
world.

Notation to specify transformations
===================================

Transformation can always be specified as 7-vectors ``Q:[p1 p2 p3 q0 q1 q2 q3]`` (position,
quaternion). But this is not intuitive for human editing. Therefore,
the bracket notation ``<...>`` allows for another notation, namely as
a chaining of little interpretable transformations, as in the old
turtle language.

Specifically, you specify a transform by chaining::
  
  t(x y z)       # translation by (x,y,z)
  q(q0 q1 q2 q3) # rotation by a quaternion
  r(r x y z)     # rotation by `r` _radians_ around the axis (x,y,z)
  d(d x y z)     # rotation by `d` _degrees_ around the axis (x,y,z)
  E(r p y)       # rotation by roll-pitch-yaw Euler angles


Joint types
===========

The ``libry.JT`` enum (in python; or rai::JointType in C++) lists all available joint type. Currently these are::

  hingeX, hingeY, hingeZ, transX, transY, transZ, transXY, trans3, transXYPhi, universal, rigid, quatBall, phiTransXY, XBall, free, tau

A quatBall is a quaternion ball joint with 4 dofs (that supports all
differentiability and optimization); a free joint is a full 7 dof
joint; a rigid joint might seem redundant, but internally it sometimes
markes a break between separate objects (like an object sitting
rigidly on a table) rather than having multiple shapes attached to the
same object.

The joint's dofs can be initialized equivalently either with a ``q``
attribute (defining the dofs values), or a ``Q`` attribute (defining
the resulting relative transformation generated by the joint).
