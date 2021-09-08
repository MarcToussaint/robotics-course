==============================
Learning about Skeletons & LGP
==============================


What is LGP?
============

LGP means Logic-Geometric Programming, which means a Mathematical
Program (=Optimization Problem) which optimizes over both, logic
decisions and continuous ("geometric") decisions.

The logic decisions determine a *Skeleton*, which basically describes
in which phases (time intervals) which constraints hold. That Skeleton
is then translated to a Mathematical Program, either over keyframes
only or the full smooth path.

To practically insight to how LGP works, there are two entry points:
* Use the lgpPlayer to explore what logical decisions are actually possible in a given scene and for given logic. The lgpPlayer (in rai/bin) takes two arguments: the folFile (first-order-logic file, g-file syntax), which describes the generic action operators (1-to-1 to PDDL), and the confFile, which describes the scene (a rai::Configuration, g-file syntax). Note that the frames in the scene description also need logical attributes so that they can appear as constants (objects) in the first order logic.
* Use the skeletonSolver to explore how a manually defined skeleton translates to a MathematicalProblem and its solution. The skeletonSolver (in rai/bin) takes two arguments: the sktFile (also of g-file syntax), which defines a skeleton in a simplified manner, where you only define a sequence of skeleton predicates that are automatically translated to phase intervals on when the predicates hold, and a confFile, which describes the scene. The skeleton symbols you use here to script a manipulation are exactly the symbols that are produced by effects of action operators in LGP.

  
Example Skeletons
=================

Let's start bottom up and consider some example skeletons. From rai/bin/src_skeletonSolver call ``x.exe``. The skeleton executed here is

.. code-block:: c++

   { [touch, r_gripper, block1], [stable_, r_gripper, block1] }
   { [above, block1, block2], [stableOn_, block2, block1] }

   { [touch, r_gripper, block2], [stable_, r_gripper, block2] }
   { [above, block2, block3], [stableOn_, block3, block2] }

   { [touch, r_gripper, block3], [stable_, r_gripper, block3] }
   { [above, block3, block4], [stableOn_, block4, block3] }

   { [touch, r_gripper, block4], [stable_, r_gripper, block4] }
   { [above, block4, block5], [stableOn_, block5, block4] }

   { [touch, r_gripper, block5], [stable_, r_gripper, block5] }
   { [above, block5, box1], [stableOn_, box1, block5] }

   { [end] }

Each list {...} is a time step and lists literals that an action operator might produce in that time step. The first argument of each literal is the symbol. Note that mode switch symbols end with _, which means that are persistent and hold until another mode symbol overwrite the mode for the object. The other symbols are geometric constraints that only hold in that time step.

This particular skeleton is very repetitive: the robot picks and places five blocks. Note that you could realize concurrent action by adding also geometric constraints and mode switches that concern the left robot in time steps.

Many things can be explore in the workshop scene: Try to let the left robot pick a stick and let it touch an object picked by the right one.



