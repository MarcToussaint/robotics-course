============================
 SimLab Exercise 2 - Motion
============================

Note: Before new exercises, always update the repo::

  cd $HOME/git/robotics-course
  git pull
  git submodule update
  cd build
  make -j4


a) Basic IK
===========

The first example in ``course3-Simulation/03-motion`` demonstrates a
minimalistic setup to use optimization for IK. The initial example
creates a KOMO instance setup to solve a 1-time-step optimization
problem (i.e., and IK problem). Start from this example to generate
more interesting motion. The specific tasks are:

1. Vary between the left gripper and right gripper reaching for the
   object. Is there a difference to "the object reaching for the right
   gripper" vs. the other way around? And test the left gripper
   reaching for the right gripper.
2. Also constrain the gripper orientation when reaching for the
   object. For a start, try to add a ``FS_quaternionDiff``
   constraint. Why does this not work immediately? Try to change the
   object pose so that the constraint can be fulfilled. Be able to
   explain the result.
3. There are (in my view better) alternatives to apriori fixing the
   gripper orientation (the full quaternion). Instead add a
   ``FS_scalarProductXZ`` constraint and try to understand. (Zoom into
   the little coordinate frame in the gripper center to understand
   conventions.) Play around with all combinations of
   ``scalarProduct??`` and understand the effect. Further, add one
   more argument ``target={.1}`` to the ``addObjective`` method, and
   understand the result. Using multiple scalar product features, how
   could you also impose a full orientation constraint, and how would
   this differ to constraining the quaternion directly?
4. Think more holistically about grasping: How could it be realized
   properly? For example, think about optimizing a series of two or
   three poses, where the first might be a so-called *pre-grasp*, and
   the others model approach and final grap (from which the
   gripper-close command could be triggered). Create such a
   sequence. Note: Do all of this without yet explicitly using
   collision (``pairCollision``) features.
5. For your information, ``tutorials/2-features.ipynb`` gives an
   impression about what alternative features one can use to design
   motion.

  
b) Path Optimization & velocity/acceleration objectives
=======================================================

The second example in ``course3-Simulation/03-motion`` demonstrates a
full path optimization example. Note the differences: We now specify the ``times`` argument when adding an objective, which is an single time slice or interval specified as a floating number. We defined the path to have 1 phase with 40 steps-per-phase; the ``times={1.}`` means the objective only holds at phase 1 (end of the path).

Further note the ``qItself`` objective with ``order=1``, which constrains the joint velocity to be zero at the end of the path.

1. Remove the qItself objective - why is the result optimal?
2. Add the qItself objective again. Additionally, constrain the
   gripper to have constant(!) acceleration (order=2) equal to
   :math:`(0,0,.1)` during the interval [0.7,1.]. What is this
   doing?
3. Modify the previous to impose a reasonable grasp approach to the
   object, that works for any orientation of the object.

c) Explore collision features, and enforce touch
================================================

So far we neglected collisions -- and it is generally a fair approach
to first try to design motions that inherently stay away from
collisions even without using collision features, as the latter imply
local optima.

The ``distance`` feature returns the *negative* distance between the
given pair of frames (where the frames need to be convex shapes). You
should impose an inequality (lower-equal zero) to force the solver to avoid
penetrations. By changing the target you can also add a margin.

1. Add an additional obstacle, e.g. a sphere, to the scene, with which
   your moving gripper (from exercise b) collides. Then add a
   ``distance`` inequality objective between the new sphere and the
   ``"R_gripper"`` object. In addition, also add the same between
   ``R_gripper`` and ``object``, which should modify the grasp
   approach. You can also try analogously for ``R_finger1`` and
   ``R_finger2``.
2. Now, remove previous grasp or collision objectives, and only add a
   ``distance`` equal to zero objective on the ``R_finger1`` and
   ``object`` as the goal constraint. This should generate a touching
   motion. This is a typical ingredient in generating pushing
   interactions.

d) Interacting with "real" objects
==================================

The two examples in ``course3-Simulation/03-motion`` do not really interact with the "real" world (Simulation, in our case), but only compute some motion in your model configuration. Let us change that:

1. Add a new object named "myObj" of shape type ``ssBox`` (see note
   below).
2. Setup a "real" world loop, and shortcut perception by always
   querying ``RealWorld["obj1"]->getPosition()`` to get the position
   of the object "obj1". Set the position of "myObj" to the same
   position.
3. Try to use a finger of either of the arms to continuously touch the
   falling object in the real world loop.

Note: ``ssBox`` means sphere-swept box. This is a box with rounded
corners. This should be your default primitive shape. The shape is
determined by 4 numbers: x-size, y-size, z-size, radius of
corners. The 2nd most important shape type is ``ssCvx`` (sphere-swept
convex), which is determined by a set of 3D points, and sphere radius
that is added to the points' convex hull. (E.g., a capsule can also be
described as simple ssCvx: 2 points with a sweeping radius.) The
sphere-swept shape primitives allow for well-defined Jacobians of
collision features.


   
e) Advanced: Tricky use of inequalities, scaling, and target
============================================================

This is a bit tricky to figure out, but if you do, you really understood the use of the scaling, target and inequalities.

As in a), consider IK for grasping a cylinder again. Let's care only
about the gripper position, not it's orientation. The position needs
to be in the interval :math:`[-l/2,l/2]` along the z-axis of the
cylinder, if it has length :math:`l`. We can model this with 4
constraints:
* The x-component of the ``positionRel(gripperCenter,object)`` needs to be equal 0
* The y-component of the ``positionRel(gripperCenter,object)`` needs to be equal 0
* The z-component of the ``positionRel(gripperCenter,object)`` needs to be lower-equal l/2
* The z-component of the ``positionRel(gripperCenter,object)`` needs to be greater-equal l/2

Can you figure out how to realize this? Tip: Choosing a scale
``arr({1, 3}, {0,0,1})`` picks out the z-component of a 3D feature (as
it means multiplication with :math:`(0,0,1)^T`.) But note, the target
always needs to live in the original 3D feature space!


e) Advanced: Reactive Operational Space Control
===============================================

Realize a simplest possible instance of Operational Space Control
using KOMO. [The python interfaces are not ready for this yet.]

1. Setup a minimal KOMO problem of order :math:`k=2`. Add a
   add_qControlObjective to penalize accelerations. Add another
   add_qControlObjective to penalize also velocities! Add a weak
   objective on the hand position. Solve and make a single step
   forward.
2. Repeat the above, always recreating KOMO from the new current
   configuration.
