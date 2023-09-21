===========================================
Learning KOMO (K-Order Markov Optimization)
===========================================


No dynamics? No velocities? How do you solve for dynamic problems?
==================================================================

The :ref:`refConfiguration` documentation describes the basic data
structures: ``Configuration``, ``Frame``, and ``Features``. But
nothing about velocities or dynamics.

In ealier versions of the code velocities where explicitly represented
in configurations, and all transformations were also dynamic
(representing also linear and angular velocities). However, influenced
by experience with KOMO, I removed these representations of dynamics
and now instead represent everything by thinking in terms of
"k-order".

Quite trivially, instead of velocities, in KOMO you only represent two
consecutive configurations. Then all velocities are implicit by the
finite difference between these two configurations. In particular,
this can be the finite difference of dofs (which represents joint
velocities), or the finite difference of any feature over the
configurations (which can represent linear or angular velocities of
frames, or velocities of shape distances, or velocities of any
numerical quantity we can compute from frames).

Representing velocites by finite difference of two concecutive
configurations might seem dull and inefficient. But it is inherently
consistent with the time discretization needed to numerically solve
dynamic problems, avoids redundancies in path optimization settings,
and generalizes directly to higher-order systems, where we might
optimize for jerk. To give a concrete example, setting up operational
space control as optimization problem over configurations, you would
instantiate 3 configurations: One for the current time :math:`t`, one
for the past time step :math:`t-\tau`, and one for the next time step
:math:`t+\tau`. The first two configurations are fixed and represent
the current dynamic state (via finite difference between :math:`t` and
:math:`t-\tau`). The thrid configuration :math:`t+\tau` is subject to
optimization and you can impose any objectives on any features, or
velocities of features, or accelerations of features, or anything
derivable from these.

What is KOMO?
=============

KOMO means k-order Markov optimization. KOMO is a way to formulate
path optimization problems. Technically, this means it is a convention
of how to specify the mathematical program, namely via the KOMO class.

It assumes that the path is represented as a sequence of T
configurations (we discretize time). The decision variable of the
mathematical program are the DOFs of each configuration. Note that the
decision variables are not velocities in time slices, but just the
"joint angles" (and other potential DOFs in a time slice). But we
can still optimize dynamical paths. The reason is that the optimizer
always considers (k+1)-tuples of consecutive configurations and all
objectives are functions of such (k+1)-tuples. This is similar to
having a k-order differential equation, instead of an ODE; or to
having a k-order Markov chain, instead of a Markov chain.

I use the word *objective* to refer to cost terms as well as
inequality and equality constraints of the mathematical program.

What is KOMO -- formally?
=========================

As a convention of notation, we describe a non-linear mathematical program as a tuple :math:`(X, \phi, \rho)`, where :math:`X` is the decision space, :math:`\phi: X \to \mathbb{R}^K` a set of features, and :math:`\rho:\{1,..,K\} \to \{\text{S},\text{C},\text{I},\text{E}\}` indicates for each feature whether it is a sum-of-squares, normal cost, inequality, or equality:

.. math::

  \min_{x\in X}~ \phi_\text{S}(x)^\top \phi_\text{S}(x) + \textbf{1}^\top \phi_\text{C}(x)
  \quad\text{s.t.}\quad \phi_\text{I}(x) \le 0,~ \phi_\text{E}(x) = 0~,

where :math:`\phi_\text{S} \equiv \phi_{\rho^{-1}(\text{S})}` is the feature
vector containing only the set of sum-of-square features, and
analogously for :math:`\phi_\text{C}, \phi_\text{I}`, and :math:`\phi_\text{E}`.

In KOMO we additionally assume that :math:`x=(x_1,..,x_T)` is a set of
variables ("factored"), and each feature :math:`\phi_j(x_{[j]})` depends
only on a subset of at most :math:`k+1` variables, where
:math:`[j] \subseteq \{1,..,T\}` indicates which variables the *j*-th feature depends on.


What are the main steps to define a KOMO problem?
=================================================

With KOMO, the steps to specify the mathematical program are 
as follows:

* First specify the basic configuration (setModel), and how many
  configurations (T) we have in this path problem. By convention, T is
  not set directly, but instead you specify a (double) number of
  phases, and steps-per-phase, the product of which gives T. The
  floating number *phase* provides you an alternative way to refer to
  time slices. In the simplest setting, you have one phase (phase is
  in [0,1]), and steps-per-phase equals T.

* Then you specify k. For smooth motions, k=2, which means we can
  define objectives on 3-tuples of consecutive configurations. For
  simple shortest path problems, we can choose k=1, where we can
  penalize the path length, but not accelerations.

* As an advanced feature (skip this first), you can then specify
  kinematic switches during the path. This means to specify operators
  that actually structurally change the kinematic tree(s) of the
  configuration at certain time slices. KOMO is designed to cope with
  such structural switches. In many kinematic reasoning settings
  these appear naturally, e.g. when an object is first considered a
  leaf of a robot arm (stable grasp), and later a leaf link of a
  tablet where it was placed on.

* Finally, and this is the main thing, you specify the list of
  objectives. KOMO has a single method for this: addObjective. The key
  to learning KOMO is to learn using this method.

Roughly, what does KOMO do with these specs?
============================================

A core of KOMO is to compile all the given information into a more
abstract representation of a non-linear mathematical program. It does
so by implementing different (virtual) problem abstractions, namely
`MathematicalProgram`, or `MathematicalProgram_Factored`. Then, behind
these problem abstractions, a solver is finding an optimal
solution. While the KOMO code comes with a default solver, this solver
should not necessarily be thought of as part of KOMO. One can
plug-and-play replace it with other general NLP solvers (currently
interfaced are NLOpt, IPOpt, and ceres).

In all cases, the full path (vector of all DOFs in all configurations)
becomes the decision variable of the solver. The difference in the
abstractions is in how much structure of the Jacobians and feature
dependencies is being exposed to the solver. In the default case (via
the MathematicalProgram abstraction), the solver does not have
detailed information about the problem structure, but gets sparse
Jacobians for all features, which makes computing Newton steps etc
computationally efficient. In the other casevia the
`MathematicalProgram_Factored` abstraction), the solver gets
information on individual variables and which objectives depend on
which variables, much like in a factor graph.

Can KOMO only optimize paths?
=============================

Actually no. You can optimize any 'network' of configurations. That
is, you can specify that there are T configurations, but they are not
necessarily consecutive. Instead, you specify objectives, but each
objective may depend only on maximally k+1 configurations - but these
are arbitrary. Thereby the structure is a network of configurations,
where the objectives define couplings or cliques of maximal size
k+1. Such problems are handled exactly the same way as path problems,
namely by exposing spare Jacobians and/or a sparse graph structure to
the solver.

Can I load an URDF?
===================

Yes. But currently it is only partially automatic. The easy part is to
convert the urdf-file to a g-file using ``$RAI/bin/urdf2rai.py``. The
resulting g-file encodes the full kinematic structure. The part which
usually requires manual fiddling are the mesh files. First, in the
g-file, you have to change the path to their location in the file
system (removing the 'package' part). Potentially that's all you
need. However, KOMO calls various collision libraries that need clean
and correct (orientation, holes, etc) mesh files. Those that come with
URDF files are typically not clean and correct. I typically use
meshlab (the command line tool) to automatically clean and compress
meshes into ply files. The best guide for this is the hubo/HOWTO.sh in
the rai-robotModels repository.

Also, you should check the results using the ``$RAI/bin/kinEdit`` tool
to display g-files from command line. CMake automatically compiles it;
otherwise call 'make bin' in $RAI to compile this. You can edit the
file with any editor during watching it with kinEdit.

First practical steps
=====================

Lesson 1: play around with $RAI/test/KOMO/tutorial
--------------------------------------------------

You can copy the folder ``$RAI/test/KOMO/tutorial`` to any place you want (e.g. your own sandbox repository). Just edit the BASE = ../.. declaration to the actual $RAI path and make will still work.

Play around with the tutorialBasics() as follows:

* in setTiming, change the steps-per-phase from 20 to 40 - which will give a more fine grained trajectory

* in setTiming, change the last argument from k=2 to k=1 - this will lead to interesting problems! (UNDO this)

* set argument setSquaredQAccVelHoming(0., 1., 0.) - which will lead
  to a shortest path (penalizing velocities) instead of a smoothest
  path (penalizing accelerations). Note that typically shortest path
  problems (also with k=1) are harder for the solver than smoothest,
  because penalizing accelerations introduces more correlations
  between configuration and a better conditioned path Hessian which
  leads to better Newton steps.

* comment-out the FS_positionDiff objective

* comment-out the FS_quaternionDiff objective

* comment-out the setSlow objective

* Play with changing objective types from OT_eq (equality constraint) to OT_sos (sum-of-squares), and reducing the scaling number {1e0} to {1e-1} or {1e-2} -- this will lead to approximate tasks only

* Play with the changing {1., -1.} to {.5, .8} for one of the objectives. This changes when the objective is active (using the phase coordinate). 0.=start, 1.=after 20 steps, 2.=after 40 steps. Specifying -1. as last entry means 'until the end'.


Lesson 2: Learn the addObjective method and the "Language of Features"
------------------------------------------------------------------------

The core of learning KOMO is to learn how to add objectives. The method declaration is

.. code-block:: c++

  struct Objective* addObjective(
     const arr& times,            //when (in phase! time) is this objective active?
	 FeatureSymbol feat,      //what feature is this about?
	 const StringA& frames,   //which frames are input to the feature?
	 ObjectiveType type,      //is this a sos, eq, or ineq objective?
	 const arr& scale=NoArr,  //an optional scaling or linear transformation! of the feature
	 const arr& target=NoArr, //an optional translation (change of zero-point) of the feature
	 int order=-1,            //how many configurations are input to the feature (order=1: velocity; order=2: acceleration)
	 int deltaFromStep=0,     //modify when exactly the objective becomes active
	 int deltaToStep=0);      //modify when exactly the objective ends being active

	 
Let's explain this with a series of examples. The first example
``addObjective({1.}, FS_position, {"endeff"}, OT_eq, {1e1},
{1.,2.,3.});`` creates a new objective for the configuration of phase
time 1. This objective is about a feature that takes a single
configuration as input and maps it to the 3D world position of the
"endeff" frame. From the resulting 3D position vector it subtracts the
vector :math:`(1,2,3)^T` (to change the zero-point to the target), and
then multiplies the vector with the scalar 1e1. The result is added to
the mathematical program as an equality constraint. As a result the
endeffector moves to the target position :math:`(1,2,3)` at phase
time 1.

The second example ``addObjective({1.}, FS_position, {"endeff"},
OT_eq, arr(2,3 {1,0,0, 0,1,0}), {1.,2.,3.});`` is almost the same as
the first, but we replaced the scalar scaling 1e1 by the
:math:`2\times 3`-matrix :math:`\begin{pmatrix}1&0&0\\0&1&0\end{pmatrix}`. The effect
is that after subtracting the target :math:`(1,2,3)^T` from the endeff
position we map the resulting 3D vector onto only the
:math:`xy`-position. This means this is an equality constraint on the
:math:`xy`-position of the endeffector to be at position
:math:`(1,2)`.

The third example ``addObjective({1.}, FS_position, {"endeff"}, OT_eq,
{1e1}, {1.,2.,3.}, 1);`` is again almost the same as the first, but we
added the order argument :math:`k=1`. This small change is a big
change of semantics: The objective does not concern the position
feature directly, but rather its finite difference between two
consecutive configuration. Therefore, this actually constrains the
velocity of the endeffector to be equal to :math:`(1,2,3)`. Below we
give more details on the semantics of time and velocities in the
section on *steps, phase, and time/. When instead we would set the
order argument to :math:`k=2`, the objective would concern the
acceleration of the endeffector position.

The fourth example is a rare case, but we add it here to also explain
the last arguments: ``addObjective({1.}, FS_position, {"endeff"},
OT_eq, {1e1}, {0,0,0}, 1, -2, +3);`` The last two arguments are a
refinement of when exactly the objective is active. In this case, the
objective not only holds at phase time 1, but starts being active 2
time discretization steps before phase time 1, and ends being active 3
time discretization steps after phase time one. The use case for this
specification is rare, but sometimes we need to be very precise in
which time steps objectives hold, esp. when imposing constraints that
relate to kinematic switches or transitions between different dynamic
phases.

Features are at the core
========================

So far we explained all arguments of addObjective except for those
that specify the feature. In the previous examples the feature was
specified by an enum symbol (FS_position) and the frame name "endeff"
to which the feature refers to. But the concept of a feature is much
more general than that. There is a second declaration of the
addObjective method,

.. code-block:: c++

  struct Objective* addObjective(const arr& times, const ptr<Feature>& f,
		ObjectiveType type,
		const arr& scale=NoArr, const arr& target=NoArr,
		int order=-1,
		int deltaFromStep=0, int deltaToStep=0);

which is identical to the
previous declaration except that, as the 2nd argument, it receives an
instance of the abstract Feature class. A Feature implements an
arbitrary differentiable mapping (the virtual phi method) from any
tuple of configurations to a vector (see the [Kin/feature.h](../blob/master/Kin/feature.h) header). In
principle the user can implement any instances of this class to define
own feature spaces in add objectives in these feature spaces to the
mathematical program. However, the KOMO code includes a large set of
predefined features, which can be referred to by the feature symbols
(see [Kin/featureSylbols.h](../blob/master/Kin/featureSylbols.h)). Most of these features are uniquely
defined by specifying which frames they refer to. Therefore, the
default is that a feature is specified by a symbol and a set of
frames. More complicated features have to be first created as a shared
pointer and then added as objective to the mathematical program.

The 3D position of a frame is one of the simplest examples of a
feature. A more complicated example is the quaternion of the relative
orientation of two frames. Another feature that is often useful is the
scalar product of two vectors, which can be attached to any two
frames. These scalar products measure alignment and are typically
constrained to be equal to 0 or 1. These are examples for geometric
features. These geometric features come in three versions: (1) the
position, vector or quaternion directly; (2) the *relative*
position of one frame as measured in another frame, the vector
attached to one frame as measured in another frame, or the quaternion
orientation of one frame as measured in another; and (3) the
*difference* in position of one frame and another as measured in
world coordinates, the difference of two vectors attached to two
frames as measured in world coordinates, or the difference of
quaternions of two frames as measured in world coordinates. Whenever
quaternions are subtracted of course this is modulo sign flip to
account for the double coverage of SO(3).

Apart from these typical geometric features there are other features,
esp. those that directly refer to the joint angles. The most
important among those is the FS_qItself feature, which is nothing but
the :math:`q`-vector itself. So this feature is basically the identity map
from DOFs to feature. The q-vector is often used as a feature, for
instance to impose acceleration sum-of-square penalties, or velocity
penalties. In fact, the high-level method setQAccVelHoming does nothing
but add up to three objectives to the KOMO problem, which concern the
joint space accelerations, joint space velocities, and a homing preference in
joint space, always imposing squared penalties. Another important
feature which refers to the joint space is the FS_qLimit feature,
which maps to a :math:`2n`-dimensional vector (for :math:`n`-dimensional :math:`q`)
which refers to the slack of all joint limits. If all :math:`2n` numbers are
negative, all joint limits are met. If one number is positive, this
means that a joint limit is violated. So the joint limit feature can
directly be used to impose an inequality objective in the mathematical
program.

Another group of features is related to collisions and shape distances
or penetrations. The FS_accumulatedCollision maps a configuration to a
single scalar that indicates penetration in the scene when the scalar
is positive. The FS_distance maps two frames to the *negative* signed
distance between their convex shapes. Thereby, imposing an inequality
on the FS_distance between two frames avoids their penetration, but
they may touch (which is important for manipulation). If you want to
avoid collision by a margin, you have to specify a negative target,
e.g., a target {-0.1} will keep the shapes 10 cm apart.

Finally, other features are related to imposing physics constraints,
and internally there are a lot more advanced features implemented,
which are not yet exposed via the simple feature symbols.

Varying decision variables per time slice: kinematic switches & force contacts
==============================================================================

KOMO is perhaps special among available trajectory optimization
methods in that is allows to optimize over sequences in which the
structure and degrees-of-freedom of configurations changes over
time. This originated from work that aimed to model kinematic
pick-and-place scenarios, where the object is initially a child of
some table without any own DOFs, then, when picked up, becomes a child
of the endeffector, with 7 *effective* DOFs that describe the constant
relative pose of the object in hand, then, when placed, becomes again
a child of a table, but now with 3 DOFs that describe the effective
placement DOFs (literally an :math:`xy\phi`-joint) between table and object.

In KOMO such kinematic switches can be added to the problem
specification equally to how objectives are added. Namely, the
addSwitch method allows this. Internally, these switches are process
quite differently to objectives: While the list of objectives defines
what information is passed to the NLP solver in each solver iteration,
the list of switches is being process *only once at the
initialization* of the optimization problem. By initialization here I
mean, the explicit creation of all the T configurations that represent
all time slices. This happens in the komo.setupConfigurations()
method, which is called in komo.reset() typically after all objectives
have been defined and just before the optimizer is started. That is,
the setupConfiguration methods accounts for all the kinematic switches
when creating the explicit kinematic representations of each time
slice.

The mechanism of introducing particular decision variables only for
certain time slides is more general than just kinematic switches. To
enable physical reasoning KOMO allows you to introduce explicit
decision variables for a potential force exchange between pairs of
frames in particular time slices of phases. This is done with the
addContact methods. While introducing new decision variables to the
mathematical program (new DOFs of the configurations in certain time
slides), these methods automatically also add a series of equality and
inequality constraints to model different kinds of contacts.

Summary: there are only a few fundamental methods in KOMO, the rest are helpers
===============================================================================

In summary, there are only a few fundamental methods to specify the optimization problem:

* setModel, to specify the blueprint configuration that is used to create configurations for all time slices (accounting for switches)
* setTiming, to specify the basic phase duration, steps, and k-order
* addObjective
* addSwitch
* addContact

Many other methods are just for convenience and internally just call the following fundamental methods. Further methods are for running the default solver, or reading out results.


Steps, phase, real time, and time optimization
==============================================

There are really *three* different quantities that relate to time in
KOMO. First, we distinguish between step and phase, where steps
enumerate the series of configurations from 0 to T-1, and phase is a
floating number from 0 to T/steps-per-phase. The precise mapping
between these is done by the conv_time2step and conv_step2time methods
in [Kin/switch.h](../blob/master/Kin/switch.h). The purpose of this distinction is that you can
design a scheme referring only to phase time, but then change your
choice of time discretization. Ideally, the mathematical program
should be almost invariant w.r.t. a change of steps-per-phase. This is
accounted for especially in the default penalizations of accelerations
and velocities in the setQAccVelHoming method: the scaling of these
squared penalties is automatically multiplied with
:math:`\sqrt{1/\text{steps-per-phase}}`.

In addition, one should distinguish between phase and real
time. First, in most practical settings, there should be a
post-processing of optimized paths to decide how fast the optimized
path should be executed in real time, e.g. by analyzing the joint
velocities in the path. Further, KOMO also allows to introduce the
real time interval :math:`\tau` between two consecutive configurations as a
decision variable subject to optimization. In fact, the
setTimeOptimization introduces a new DOFs to each configuration (a
``0th joint'') which represents the :math:`\tau` relative to the previous
configuration. All of these are then also optimized. The
setTimeOptimization automatically also adds objectives to the
mathematical program which ensure that :math:`\tau` is lower bounded and
evolves smoothly. This tau is actually used to evaluate real time
accelerations and velocities in the physics related features, in
particular in the NewtonEuler equation constraint. And it is
especially in physics applications where certain sub-motions cannot be
aligned with a prefixed time schedule. In this case it is essential to
allow KOMO to adapt the real time evolution so that dynamic
motions can align with a prefixed phase schedule.




LINKS to more documentation
===========================

* And old arxiv tech report: https://arxiv.org/abs/1407.0414
* docu links in rai-maintenance: https://github.com/MarcToussaint/rai-maintenance/tree/master/help
* docu of features for the python bindings: https://github.com/MarcToussaint/rai-python/blob/master/docs/2-features.ipynb
