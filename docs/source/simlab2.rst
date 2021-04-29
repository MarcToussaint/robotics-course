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
   gripper" vs.\ the other way around? And test the left gripper
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

  
b) Path Optimization for a 3-phase handover
===========================================

[details to come by Jung-Su]

Essentially, use KOMO in full path optimization mode, with 3 phases
and 20 steps per phase, to model a pick, hand-over, and target
placement, as demonstrated by Jung-Su.


a) Compute a 2-arm robot configuration, where the graspCenter
   positions of both hands coincide, the two hands oppose, and their
   x-axes are orthogonal. (E.g., as if they would handover a little
   cube.)
b) Add a box (shape type ``ssBox``, see Tip2 below) somewhere to the
   scene, compute a robot configuration where one of the grippers
   grasps the box (centered, along a particular axis), while avoiding
   collisions between the box and the two fingers and gripper.
c) Propose alternatives for how to design grasps, based on any
   geometric feature you can think of. (Note that our collision code
   can compute normals and witness points for proximity queries, which
   can be used.)

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

Concretely:
* Add an additional obstacle, e.g. a sphere, to the scene, with which your moving object (from exercise b) collides. Then add a ``distance`` feature between the new sphere and the moving object.
* Completely independent from exercise b), generate a simple object reaching motion, where the goal objective is to make the ``gripper`` shape touch the object -- by imposing an equality constraint on their distance.

The latter is a typical ingredient in generating pushing interactions.


d) Tricky use of inequalities, scaling, and target
==================================================

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

a) Setup a minimal KOMO problem of order :math:`k=2`. Add a
   add_qControlObjective to penalize accelerations. Add another
   add_qControlObjective to penalize also velocities! Add a weak
   objective on the hand position. Solve and make a single step
   forward.
b) Repeat the above, always recreating KOMO from the new current
   configuration.
