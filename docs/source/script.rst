======
Script
======


Introduction
============

Reference material
------------------

In terms of background, please refer to the `Maths for Intelligent
Systems <https://www.user.tu-berlin.de/mtoussai/teaching/Lecture-Maths.pdf>`__
as well as the `Intro to
Robotics <https://www.user.tu-berlin.de/mtoussai/teaching/Lecture-Robotics.pdf>`__
lecture scripts. Here a list of further teaching material:

-  Craig, J.J.: *Introduction to robotics: mechanics and control*.
   Addison-Wesley New York, 1989. (3rd edition 2006)

-  Steven M. LaValle: *Planning Algorithms*. Cambridge University Press,
   2006.

   **online:** http://planning.cs.uiuc.edu/

-  VideoLecture by Oussama Khatib:
   http://videolectures.net/oussama_khatib/

   (focus on kinematics, dynamics, control)

-  Oliver Brock’s lecture
   http://www.robotics.tu-berlin.de/menue/teaching/

-  Stefan Schaal’s lecture Introduction to Robotics:
   http://www-clmc.usc.edu/Teaching/TeachingIntroductionToRoboticsSyllabus

   (focus on control, useful: Basic Linear Control Theory (analytic
   solution to simple dynamic model :math:`\to` PID), chapter on
   dynamics)

-  Chris Atkeson’s “Kinematics, Dynamic Systems, and Control”
   http://www.cs.cmu.edu/~cga/kdc/

   (uses Schaal’s slides and LaValle’s book, useful: slides on 3d
   kinematics http://www.cs.cmu.edu/~cga/kdc-10/ewhitman1.pptx)

-  CMU lecture “introduction to robotics”
   http://www.cs.cmu.edu/afs/cs.cmu.edu/academic/class/16311/www/current/

   (useful: PID control, simple BUGs algorithms for motion planning,
   non-holonomic constraints)

-  *Springer Handbook of Robotics, Bruno Siciliano, Oussama Khatib*
   http://link.springer.com/book/10.1007/978-3-319-32552-1

-  LaValle’s *Planning Algorithms* http://planning.cs.uiuc.edu/

Coding Getting Started
----------------------

The coding secitons use the @robotic@ python package. Sources and
dockers to build the wheel are found on
`github/rai-python <https://github.com/MarcToussaint/rai-python>`__.

To install on a standard Ubuntu, the following should be sufficient

::

    sudo apt install liblapack3 freeglut3 libglew-dev python3 python3-pip
    python3 -m pip install --user robotic numpy scipy

A standard test is

::

    python3 -c 'from robotic import ry; ry.test.RndScene()'

Several `jupyter example
notebooks <https://github.com/MarcToussaint/robotics-course/blob/master/course4-Panda/script2-IK.ipynb>`__
are available as part of the
`github/robotics-course <https://marctoussaint.github.io/robotics-course/>`__.

Scene & Robot Description
=========================

Generally speaking, a scene is a collection of objects (including robot
parts). We typically assume objects to be rigid bodies with fixed shape
– which clearly is a simplification relative to real world. More about
this below, in section [secShapes].

However, formally we define a scene as a collection of **frames**, which
is short for coordinate frames. We can think of these frames as oriented
locations in 3D space – and various things can be associated to these
frames. If a rigid shape and mass is associated to a frame, then it
makes a typical rigid body. But frames can also be associated to robot
joint locations or virtual landmarks.

Transformations
---------------

Let :math:`i=1,..,m` enumerate :math:`m` frames in a scene. Each frame
has a **pose** :math:`X_i\in SE(3)`, where
:math:`SE(3) = {{\mathbb{R}}}^3 \times SO(3)` is the group of 3D
transformations, namely the cross-product of translations and rotations.
We always assume a world origin to be defined and use the word *pose*
specifically for the transformation from world origin to the object
frame.

Transformations in :math:`A\in SE(3)` are tuples :math:`A = (t, r)`,
where :math:`t\in{{\mathbb{R}}}^3` is a translation and
:math:`r\in SO(3)` a rotation – see Appendix [appTransforms] for more
details. Rotations can be represented as matrix :math:`R` (see the Maths
script on properties of rotation matrices), and a pose as the
:math:`4\times 4` homogeneous transform
:math:`{ \left(\begin{array}{cc}R & t \\ 0 & 1\end{array}\right) }`.
However, more commonly in code we represent rotations as a 4D quaternion
:math:`r\in{{\mathbb{R}}}^4` with unit length :math:`|r| = 1`. I always
use the convention :math:`r=(r_0,\bar r)`, where the first entry
:math:`r_0 = \cos({\theta}/2)` relates to the total rotation angle
:math:`{\theta}`, and the last three entries
:math:`\bar r = \sin({\theta}/2)~ \underline w` relate to the unit
length rotation axis :math:`\underline w`.

Euler angles and the scaled rotation vector are alternative rotation
representations – but never use them.

See appendix [appTransforms] for reference and conversions.

Let’s create a mini scene of 2 frames, both have no proper shape but
just a “marker” displaying their axes. We create a yaml-style ``mini.g``
file:

::

    A: { X: [1,0,1, 1,0,0,0], shape: marker, size: [.5] }
    B: { X: [-1,0,1, 1,0,0,0], shape: marker, size: [.3] }

Here, ``X`` provides the pose of frame :math:`A` and :math:`B`, given
with 7 numbers, which are translation and quaternion. Let’s load and
display it:

::

    from robotic import ry
    C = ry.Config()
    C.addFile('mini.g')
    C.view()

and you’ll see the two markers in places :math:`(1,0,1)` and
:math:`(-1,0,1)`.

Providing quaternions is rather non-intuitive for a human. So there is a
second convention of how to specify transformations in a more
human-intuitive manner with turtle commands:

::

    A: { X: "t(1 0 1) d(30 1 0 0)", shape: marker, size: [.5] }
    B: { X: "t(-1 0 1) d(90 1 0 0)", shape: marker, size: [.3] }

where the string is interpreted as sequential commands of translation
(t), and rotation by degrees around an axis (d). These can be sequenced
to specify the full transform more intuitively. Also Euler angle
commands (E), rotation by radians (r), and a quaternion (q) are
interpreted. Play around with this and see how the frame poses change.
To reload you can also try calling

::

    C.watchFile('mini.g')

which is a helper that automatically updates the display whenever the
file changes – but doesn’t always handle file syntax errors well.

Coordinates and Composition of Transformations
----------------------------------------------

.. figure:: geo-transforms-2
   :alt: [figTransforms] Composition of transforms.

   [figTransforms] Composition of transforms.

Consider Fig. [figTransforms], were we have three frames :math:`1,2,3`
in addition to the world origin frame :math:`W`. Each frame has a global
pose :math:`X_1, X_2, X_3`, and relative transforms
:math:`Q_{W\to 1}, Q_{1\to 2}, Q_{2\to 3}`. We have

.. math::

   \begin{aligned}
   X_1 &= Q_{W\to 1} \\
   X_2 &= Q_{W\to 1} \circ Q_{1\to2} \\
   X_3 &= Q_{W\to 1} \circ Q_{1\to2} \circ Q_{1\to3} ~.\end{aligned}

\ Note that when composing relative transforms, we concatenate (append)
them *on the right*! Intuitively, this describes a concatenation of
turtle commands, where a turtle is initially placed on the world origin,
then translates, then rotations, then translates *relative to its own
pose*, then rotations *relative to its own pose*, etc, and ends up in
pose :math:`X_3`.

Now consider the position of a point in 3D space. It can be given in
world coordinates :math:`x^W`, but also in relative coordinates
:math:`x^1, x^2, x^3`. We have

.. math::

   \begin{aligned}
   x^W &= Q_{W\to 1}~ Q_{1\to2}~ Q_{1\to3}~ x^3 = X_3~ x^3 ~.\end{aligned}

Now you might want to ask: “does :math:`Q_{1\to 2}` describe the forward
or the backward transformation from frame :math:`1` to frame :math:`2`?”
But this question is somewhat ill-posed. The situation is:

-  :math:`Q_{1\to 2}` describes the translation and rotation of *frame*
   :math:`2` *relative* to :math:`1`. So you may call it the “forward
   FRAME transformation”.

-  :math:`Q_{1\to 2}` describes the coordinate transformation from
   :math:`x^2` to :math:`x^1 = Q_{1\to 2} x^2`. So you may call it the
   “backward COORDINATE transformation”.

In the view of fundamental linear algebra, this should not surprise as
vectors (and frames) transform *covariant*, while coordinates transform
*contra-variant*. See the maths lecture.

Let’s make the ``mini.g`` scene to represent a sequence of frames:

::

    A: { X: "t(1 0 1) d(30 1 0 0)", shape: marker, size: [.5] }
    B(A): { Q: "t(-1 0 1) d(90 1 0 0)", shape: marker, size: [.3] }
    C(B): { Q: "t(0 0 .5) d(30 1 0 0)", shape: marker, size: [.3], color:[1 0 0] }

Relative to the above, we made :math:`B` a frame *relative* to
:math:`A`, i.e., :math:`B` is now a child of :math:`A`, and added
another child :math:`C` of :math:`B`. Note that instead of specifying
the pose :math:`X`, we now have to specify the relative transform
:math:`Q`.

Scene Tree or Forest
--------------------

Scenes are typically represented as trees, with the world origin as a
root, and the pose of children specified by a *relative* transformation
from the parent. For instance, a scene with a book on a table on the
ground on the world, would have four frames with poses
:math:`X_0, X_1, X_2, X_3` (of the world, ground, table, book), but the
scene would typically be represented by relative transforms
:math:`Q_1, Q_2, Q_3` such that

.. math:: X_i = X_{i{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}} \circ Q_i ~.

\ Note that every frame can only have a single parent, and we can
abbreviate the notation :math:`Q_i \equiv Q_{\text{parent}(i)\to i}`.

Scenes can also be a forest of frames, where some frames have no parent
and their pose :math:`X_i` must be specified, while for non-roots the
relative transform :math:`Q_i` is specified. We usually only talk about
trees, but include meaning forests.

Kinematics
==========

Robots as Parameterized Trees
-----------------------------

The key thing in robotics is that some relative transforms (between
robot links) are “motorized” and can be moved. Formally, this means that
*some* of the relative transforms :math:`Q_i` in our scene have
**degrees of freedom** (dof) :math:`q_i \in {{\mathbb{R}}}^{d_i}`.

For typical robots (with hinge or linear joints) each :math:`q_i` is
just a single number (the joint dimensionality :math:`d_i=1`). E.g., a
**hinge** joint around the (local) :math:`x`-axis has a single dof
:math:`q_i\in{{\mathbb{R}}}` that parameterizes the relative transform

.. math::

   \begin{aligned}
   Q_i(q_i) = { \left(\begin{array}{cccc}
   1 & 0 & 0 & 0 \\
   0 & \cos(q_i) & -\sin(q) & 0 \\
   0 &  \sin(q_i) & \cos(q) & 0 \\
   0 & 0 & 0 & 1\end{array}\right) } ~.\end{aligned}

\ And a **prismatic** (or translational) joint along the (local)
:math:`x`-axis parameterizes

.. math::

   \begin{aligned}
   Q_i(q_i) = { \left(\begin{array}{cccc}
   1 & 0 & 0 & q \\
   0 & 1 & 0 & 0 \\
   0 & 0 & 1 & 0 \\
   0 & 0 & 0 & 1\end{array}\right) } ~.\end{aligned}

\ Other joint types (universal, cylindrical) are less common.

A bit special are **ball (spherical) joints**: They parameterize
arbitrary rotations within :math:`Q_i` – in principle they could be
described as having 3 dofs (as the Lie group :math:`SO(3)` is a 3D
manifold), however, in code it is practice to again use quaternions to
parameterize rotations, which means :math:`q_i\in{{\mathbb{R}}}^4` for
ball joints. However, note that this is an over parameterization: If
:math:`q_i` is externally “set” by a user or some algorithm, it may not
(exactly) be normalized but :math:`Q_i(q_i)` is defined to be the proper
rotation that corresponds to the quaternion :math:`q_i/|q_i|`. Note that
if a user or algorithms sets such a quaternion parameter to zero, that’s
a singularity and strict error.

In the scene tree, some of the relative transforms :math:`Q_i` are
parameterized by dofs, :math:`Q_i(q_i)`. Note that
:math:`X_\text{parent$(i)$}` is the **joint base frame**, i.e.,
determines the location and orientation of the joint axis, while
:math:`X_i = X_\text{parent$(i)$} Q_i` is the **joint output frame**. In
a robot structure one typically has chains of alternating rigid and
parameterized transforms, e.g.,

a rigid transform :math:`Q_{\pi(i)}` from world into the base of joint
:math:`i`

a parameterized transform :math:`Q_i(q_i)` representing the joint motion

a rigid transform :math:`Q_{i \to \pi(j)}` from the output of :math:`i`
into the base of a following joint :math:`j`

a parameterized transform :math:`Q_j(q_j)`

etc

There is a minimalistic convention of describing robot structures,
called Denavit-Hartenberg convention. These describe the rigid
transformations between joints using only 4 numbers instead of 6 (which
pre-determines the zero calibration as well as the “lateral” positioning
of the following joint base frame). But there is no need to use this
convention and the above notation is conceptually cleaner and leads to
intuitive, freely user-defined joint base frames.

Let’s make our ``mini.g`` scene a robot:

::

    A: { X: "t(1 0 1) d(30 1 0 0)", shape: marker, size: [.5] }
    B(A): { joint: hingeX, Q: "t(-1 0 1) d(90 1 0 0)", shape: marker, size: [.3] }
    C(B): { Q: "t(0 0 .5) d(30 1 0 0)", shape: marker, size: [.3], color:[1 0 0] }

Frame :math:`B` became a joint. Note that :math:`A` is the joint base
frame that determines its location and orientation, :math:`B` is the
joint (output) frame, and :math:`C` is a down-stream frame attached to
:math:`B`.

If you use ``watchFile`` and hit ENTER, the configuration is animated so
that the joint is articulated between its limits. In our case this leads
to a strange effect, uncovering an issue with our scene description:
Frame :math:`B` is specified with a relative transform :math:`Q` that
includes a translation; but it is also specified as ``hingeX`` joint
which can only generate rotations about :math:`x`. The two
specifications are inconsistent and we should remove the direct
:math:`Q` specification:

::

    A: { X: "t(1 0 1) d(30 1 0 0)", shape: marker, size: [.5] }
    B(A): { joint:hingeX, q: 0.5, limits: [-2., 2], shape: marker, size: [.3] }
    C(B): { Q: "t(0 0 .5) d(30 1 0 0)", shape: marker, size: [.3], color:[1 0 0] }

Here we also specified an initial joint angle :math:`q=0.5` for the
hinge joint, as well as limits.

Configuration & Joint Vector
----------------------------

We use the word **configuration** for an “articulated scene”, i.e.,
where some relative transforms :math:`Q_i(q_i)` are parameterized by
dofs :math:`q_i \in {{\mathbb{R}}}^{d_i}` (and also other dofs such as
forces or timings might be represented). A configuration can include
multiple robots – from our perspective there is no difference between
one or multiple robots. It’s just a parameterized forest of frames.

We define the **joint vector** :math:`q\in{{\mathbb{R}}}^n` to be the
stacking of all dofs :math:`q_i` (all dofs of a configuration). Given
the joint vector, we can forward chain all relative transformations in
the scene and thereby compute the absolute pose :math:`X_i(q)` of every
frame as a function of :math:`q`.

Forward Kinematics
------------------

This function :math:`q \mapsto X_i(q)` is the core of **forward
kinematics**. It describes how the joint vector :math:`q` determines the
pose of all frames in the configuration.

The precise definition of the term **forward kinematics** varies across
textbooks. I find the most concise definition to be the mapping from all
dofs :math:`q` to the full configuration state
:math:`\{X_i(q)\}_{i=1}^m`, which so far we described in terms of all
frame poses. This definition is consistent with the formal description
of *kinematics* as the theory of possible motions of a system
configuration (see [secKinematics]).

But in practice, the word forward kinematics is often used simply as the
mapping from :math:`q` to one particular “feature” of the configuration.
For instance, if :math:`X_i(q)=(t_i(q),r_i(q))` is the pose of some
frame :math:`i`, forward kinematics can describe the mapping

-  :math:`q\mapsto t_i(q)`   to the position of frame :math:`i`

-  :math:`q\mapsto r_i(q) \textbf{e}_x`   to the :math:`x`-axis of frame
   :math:`i` (where :math:`\textbf{e}_x = (1,0,0)^{{\!\top\!}}`).

-  :math:`q\mapsto X_i(q) p`   to the world coordinate a point attached
   to frame :math:`i` with fixed relative offset :math:`p`.

Each of these are 3-dimensional features. Let specifically focus on
three basic feature definitions

.. math::

   \begin{aligned}
   q \mapsto \phi^{{\textsf{pos}}}_{i,p}(q) &= X_i(q)~ p \quad\in {{\mathbb{R}}}^3 ~, \\
   q \mapsto \phi^{{\textsf{vec}}}_{i,v}(q) &= r_i(q)~ v \quad\in {{\mathbb{R}}}^3 ~, \\
   q \mapsto \phi^{{\textsf{quat}}}_{i}(q) &= r_i(q) \quad\in {{\mathbb{R}}}^4 ~,\end{aligned}

\ where :math:`\phi^{{\textsf{pos}}}_{i,p}(q)` is the (world) position
of a point attached to frame :math:`i` with relative offset :math:`p`,
:math:`\phi^{{\textsf{vec}}}_{i,v}(q)` is the world coordinates of a
vector :math:`v` attached to frame :math:`i`, and
:math:`\phi^{{\textsf{quat}}}_{i}(q)` is the 4D quaternion orientation
of frame :math:`i`. From these three, many others features can be
derived.

E.g., also the :math:`3\times 3` rotation matrix is a useful basic
feature (as it is often used in equations). We can easily construct it
by concatenating columns, :math:`\phi^{{\textsf{rot}}}_i =
(\phi^{{\textsf{vec}}}_{i,e_x}, \phi^{{\textsf{vec}}}_{i,e_y}, \phi^{{\textsf{vec}}}_{i,e_z}) \in {{\mathbb{R}}}^{3\times
3}` for basis vectors :math:`e_x,e_y,e_z` of frame :math:`i`. Note that
the Jacobian of this is a :math:`3\times 3 \times n` tensor.

The output space of the kinematic map is also called **task space**.
However, I often just call it **kinematic feature**.

In our mini scene we can get and set the joint state, as well as query
the pose of all frames:

::

    q = C.getJointState()
    print(q)

    q[0] = q[0] + .5
    C.setJointState(q)
    C.view()

    frameC = C.frame('C')
    print('pos:', frameC.getPosition(), 'quat:', frameC.getQuaternion())

    q[0] = q[0] + .5
    C.setJointState(q)
    print('pos:', frameC.getPosition(), 'quat:', frameC.getQuaternion())

This example directly accesses a frame to query its position and
orientation. Frames can also be created and modified in this way.
However, below we introduce a more abstract way to access *features*
that is more consistent to how constraint problems are formulated.

Jacobians
---------

We will use kinematic features :math:`\phi` to formulate differentiable
constraint and optimization problem. Therefore, we assume all kinematic
features :math:`\phi` are differentiable and we can efficiently compute
the **Jacobian**

.. math::

   \begin{aligned}
   J(q) = \frac{{\partial}}{{\partial}q}\phi(q) ~.\end{aligned}

\ If :math:`y = \phi(q)`, then this Jacobian tells us how a velocity
:math:`\dot q` in joint space implies a velocity :math:`\dot y` in task
space,

.. math::

   \begin{aligned}
   \dot y = J(q) \dot q ~.\end{aligned}

\ Recall that the forward kinematics is essentially implemented by
forward chaining the relative transforms :math:`Q_i`. If we use an
auto-differentiable programming language for this, we’d directly have
the Jacobians. However, the Jacobians can also directly be expressed
analytically and their computation turns out simpler and more efficient
than the forward chaining itself. To implement a kinematic engine we
essentially need to figure out how the different joint types contribute
to the Jacobians of the three basic features above. This is covered by
considering the following cases:

Rotational Joint
~~~~~~~~~~~~~~~~

Consider that somewhere on the path from world to frame :math:`i` there
is a rotational (hinge) joint :math:`j` positioned at :math:`p_j` and
with unit axis vector :math:`a_j` (both in world coordinates). Now
consider a point attached to frame :math:`i` at world coordinate
:math:`p`. (Note that we needed forward kinematics to determine
:math:`p_j, a_j`, and :math:`p`.) Then the velocity :math:`\dot p`
relates to the joint angle velocity :math:`\dot q_j` by

.. math:: \dot p = [a_j \times (p - p_j)]~ \dot q_j ~.

\ Now assume a vector :math:`v` attached to frame :math:`i`. Its
velocity is

.. math:: \dot v = [a_j \times v]~ \dot q_j = [-{{\text{skew}}}(v)~ a_j]~ \dot q_j ~.

\ Now consider the quaternion :math:`r_i` of frame :math:`i`. Its
velocity (much less obvious, see appendix Eq. ([eqQuatVel])) is

.. math:: \dot r_i = {{\frac{1}{2}}}[(0,a_j)\circ r_i]~ \dot q_j ~.

Recall that :math:`q\in{{\mathbb{R}}}^n` is the full joint vector. Let
:math:`j` be the dof index of our rotational joint such that
:math:`q_j \in {{\mathbb{R}}}` is the scalar joint angle. Further, let
:math:`p_j,a_j` be the joint position and axis, and :math:`p` a world
query point. We define two matrices that are zero except for the
provided columns:

.. math::

   \begin{aligned}
   J^{{\textsf{ang}}}\in {{\mathbb{R}}}^{3 \times n} \quad\text{with}\quad &J^{{\textsf{ang}}}_{:,j} = a_j ~, \\
   J^{{\textsf{pos}}}(p) \in {{\mathbb{R}}}^{3 \times n} \quad\text{with}\quad &J^{{\textsf{pos}}}_{:,j} = a_j \times (p - p_j) ~.\end{aligned}

\ With these two matrices we can rewrite the above equations as

.. math::

   \begin{aligned}
   \dot p &= J^{{\textsf{pos}}}(p)~ \dot q \\
   \dot v &= [-{{\text{skew}}}(v)~ J^{{\textsf{ang}}}(p)]~ \dot q \\
   \dot r &= {{\frac{1}{2}}}[\text{Skew}(r)~ \bar J^{{\textsf{ang}}}(p)]~ \dot q \quad\text{where}\quad \text{Skew}(w,x,y,z) =
   { \left(\begin{array}{cccc}
      +w & -x & -y & -z \\
      +x & +w & +z & -y \\
      +y & -z & +w & +x \\
      +z & +y & -x & +w\end{array}\right) } ~, \label{eqQuatRate}\end{aligned}

\ where by convention the cross-product :math:`[A\times v]` for a
:math:`3\times n` matrix with a 3-vector takes the cross-products
*row-wise* (could perhaps better be written :math:`[-v\times A]`). The
last equation is derived in the appendix with Eq. ([eqQuatVel]), where
we discuss how an angular velocity translates to a quaternion velocity.
The bar in :math:`\bar J^{{\textsf{ang}}}` makes this a
:math:`4\times n` matrix by inserting a zero top row (analogous to
:math:`(0,w)` in ([eqQuatVel])). The :math:`\text{Skew}` is an unusual
definition of a skew matrix for quaternions, so that quaternion
multiplication :math:`a \circ b` can be written linearly as
:math:`\text{Skew}(b)~ a`.

Now, if in our scene tree we have more than one rotational joint between
world and frame :math:`i`, each of these joints simply contribute
non-zero columns to our basic matrices
:math:`J^{{\textsf{ang}}}, J^{{\textsf{pos}}}(p)`. So this is the core
of what we have to implement for rotational joints.

Translational Joint
~~~~~~~~~~~~~~~~~~~

A translational (prismatic) joint on the path from world to frame
:math:`i` also contribute a column to the basic matrix
:math:`J^{{\textsf{pos}}}(p)`, but contributes notion to
:math:`J^{{\textsf{ang}}}` (as it does not imply rotational velocity in
the sub-branch). Specifically, let :math:`a_j` the translational axis of
the joint with dof index :math:`j`, then it simply contributes a column

.. math::

   \begin{aligned}
   J^{{\textsf{pos}}}_{:,j} = a_j ~.\end{aligned}

That’s it for translational joints.

Quaternion Joint
~~~~~~~~~~~~~~~~

Trickier, but important for ball and free joints is to also know how a
quaternion joint contributes columns to :math:`J^{{\textsf{ang}}}` and
:math:`J^{{\textsf{pos}}}(p)`. Modifying a quaternion parameterization
:math:`q_j\in{{\mathbb{R}}}^4` of a relative transform :math:`Q_j(q_j)`
implies in some way a rotational velocity down the branch. So the effect
should be similar to a rotational joint, but without fixed axis and
modulated by the normalization of :math:`q_j`. The solution is derived
in the appendix with Eq. ([eqQuatJac]) and summarized here: Let
:math:`X_j` be the *output* pose of the quaternion joint. (Yes, output!)
And let :math:`R_j` be the :math:`3\times 3` rotation matrix for the
world pose :math:`X_j`, and let :math:`r_j \in {{\mathbb{R}}}^4` be the
quaternion of the *relative* joint transform :math:`Q_j`. Then

.. math::

   \begin{aligned}
   \label{eqQuatJoint1}
   J^{{\textsf{ang}}}_{:,j} = \frac{1}{|q|} R_j J(r_j) {~,\quad}\text{where}\quad
   J(r)_{:,k} &= -2 (e_k \circ r^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}})_{1:3} ~.\end{aligned}

\ Here, :math:`e_i` for :math:`k=0,..,3` are the unit quaternions and
the matrix :math:`J(r)\in{{\mathbb{R}}}{3 \times 4}` describes how a
variation of a quaternion :math:`r` induces a 3D rotation vector
relative to the *output* space of :math:`r`. I call this the quaternion
Jacobian. The derivation is found in the appendix when discussion how a
quaternion velocity implies and angular velocity. The multiplication
with :math:`R_j` transforms this rotation vector to world coordinates.
The division by :math:`|q_j|` accounts when the dof :math:`q_j` is not
(exactly) normalized.

As we figured out the angular vector induced by a variation of a
quaternion joint, this also defines the column it contributes to the
positional Jacobian:

.. math::

   \begin{aligned}
   J^{{\textsf{pos}}}_{:,j}(p) = [\frac{1}{|q|} R_j J(r_j)] \times (p - p_j) ~,\end{aligned}

\ where :math:`p_j` is the position of the quaternion joint.

Note how differently we treat the quaternion :math:`q_j` as a joint
parameterization :math:`Q_j(q_j)` and the quaternion :math:`r_i` as a
kinematic (“output”) feature of frame :math:`i`. For instance, we can
have the Jacobian of the quaternion :math:`r_i` w.r.t. the quaternion
joint parameterization :math:`q_j`, by inserting ([eqQuatJoint1]) into
([eqQuatRate]). And even if all other transformation in the scene are
identities and the output quaternion :math:`r_i` is “essentially
identical” to the joint quaternion :math:`q_j`, the Jacobian is still
not exactly identity, as it accounts for normalization (and potential
flip of sign).

Implementing a Kinematic Engine
-------------------------------

The above provides all essentials necessary to implement a rather
general kinematic engine. To summarize:

-  Represent a scene configuration as a tree of frames, where for each
   frame we store the absolute pose :math:`X` and relative transform
   :math:`Q`. We also annotate which relative transforms :math:`Q` have
   dofs and how many. We need to maintain an index mapping that tells us
   which entries :math:`q_j` of the full joint vector parameterize a
   given relative transformation :math:`Q_j(q_j)` (essentially mapping
   between :math:`q`-indices and frame indices).

-  An efficient implementation of forward chaining transformations:
   Given the absolute poses :math:`X` of all root frames and all
   relative transforms :math:`Q`, implement an efficient algorithm to
   forward chain transformations to ensure any :math:`X_i`. Do this
   lazily on demand: Only when an absolute frame :math:`X_i` is actually
   queried call this forward chaining for this :math:`X_i` only.

-  An efficient implementation of the matrices
   :math:`J^{{\textsf{pos}}}` and :math:`J^{{\textsf{ang}}}`, which, for
   any query frame :math:`i`, determines which joints are on the path
   from :math:`i` to a root frame and for each of these joints
   contributes the corresponding columns to :math:`J^{{\textsf{pos}}}`
   and :math:`J^{{\textsf{ang}}}`. To account for large systems
   (esp. path configurations, see below) matrices should be returned in
   sparse format.

Based on this, one provides more convenient user functions that allow to
query kinematic features for any frame :math:`i`, including the pose
:math:`X_i`, and on demand also provide the Jacobian of that feature.

All features and Jacobians can be accessed on a Configuration via the
``eval`` method, which should be preferred over directly accessing
frames:

::

    [y,J] = C.eval(ry.FS.position, ['C'])
    print('position of C:', y, '\nJacobian:', J)
    type(J)

    [y,J] = C.eval(ry.FS.quaternion, ['C'])
    print('quaternion of C:', y, '\nJacobian:', J)

Here, ``FS.position`` is a feature symbol, see ``help(ry.FS)`` for a
list of all exposed features symbols. (The C++ includes more feature
implementations, and one can easily add own definitions of novel
features.) The quaternion and vector features we mentioned above are
available, but also many more that compute relative features.

The full signature of the eval method is
``eval(FeatureSymbol, frames, scale, target, order)``, with arguments:

A symbol for the feature function, e.g. relative position.

A list of frame names (strings) that indicate where in the configuration
the feature is evaluated.

An arbitrary scaling matrix :math:`S` (which can also map a
:math:`d`-dimensional feature to a :math:`d'<d` feature). (Syntactic
sugar: :math:`[]=1, [s] = {{\rm diag}}(s)`.)

A target :math:`y^*\in{{\mathbb{R}}}^d` (which is subtracted *before*
multiplication with :math:`S`).

And a differentiation order – only relevant for path configurations
explained later.

For instance, ``(positionRel, [A,B],`` :math:`S`, :math:`y^*`) would
define a re-scaled feature

.. math:: S~ [\phi^\text{positionRel}_{A, B}(q) - y^*] ~.

\ Including a scaling and target offset into the specification of a
feature will make it simple to use them as basic building blocks to
define mathematical programs, i.e., cost or equality or inequality
constraints. A specific example to only get the :math:`z`-position
relative to a target 0.5:

::

    C.eval(ry.FS.position, ['C'], [[0,0,1]], [0,0,0.5])

Features in rai
~~~~~~~~~~~~~~~

[FUSE WITH ABOVE]

We assume a single configuration :math:`q`, or a whole set of
configurations :math:`\{q_1,..,q_T\}`, with each
:math:`q_i \in\mathbb{R}` the DOFs of that configuration.

A feature :math:`\phi` is a differentiable mapping

.. math:: \phi: q \mapsto \mathbb{R}^D

\ of a single configuration into some :math:`D`-dimensional space, or a
mapping

.. math:: \phi: (q_0,q_2,..,q_k) \mapsto \mathbb{R}^D

\ of a :math:`(k+1)`-tuple of configurations to a :math:`D`-dimensional
space.

The rai code implements many features, most of them are accessible via a
feature symbol (FS). They are declared in
`featureSymbols.h <https://github.com/MarcToussaint/rai/blob/master/rai/Kin/featureSymbols.h>`__.
New features can be implemented by overloading the abstract Feature
class.

Table [tabFeatures] lists feature symbols with the respective
dimensionality :math:`D`, the default order :math:`k`, and a
description. A feature is defined by

The feature symbol (``FS_...`` in cpp; ``FS....`` in python)

The set of frames it refers to

Optionally: A target, which changes the zero-point of the features
(optimization typically try to drive features to zero, see below)

Optionally: A scaling, that can also be a matrix to down-project a
feature

Optionally: The order :math:`k`, which can make the feature a velocity
or acceleration feature

Target and scale redefine a feature to become

.. math:: \phi(q) \gets \texttt{scale} \cdot (\phi(q) - \texttt{target})

\ The target needs to be a :math:`D`-dim vector. The scale can be a
matrix, which projects features; e.g., and 3D position to just
:math:`x`-position.

The order of a feature is usually :math:`k=0`, meaning that it is
defined over a single configuration only. :math:`k=1` means that it is
defined over two configurations (1st oder Markov), and redefines the
feature to become the difference or velocity

.. math:: \phi(q_1,q_2) \gets \frac{1}{\tau}(\phi(q_2) - \phi(q_1))

\ :math:`k=2` means that it is defined over three configurations (2nd
order Markov), and redefines the feature to become the acceleration

.. math:: \phi(q_1,q_2,q_3) \equiv \frac{1}{\tau^2}(\phi(q_1) - 2 \phi(q_2) + \phi(q_3))

Inverse Kinematics
------------------

.. figure:: marionette-Tasks-2
   :alt: We can “puppeteer” a robot by defining optimization problems
   with task space constraints and solve for the joint state.

   We can “puppeteer” a robot by defining optimization problems with
   task space constraints and solve for the joint state.

We introduced forward kinematics as a mapping from an
:math:`n`-dimensional joint vector :math:`q\in{{\mathbb{R}}}^n` to some
:math:`d`-dimensional kinematic feature
:math:`y=\phi(q) \in{{\mathbb{R}}}^d`. Inverse kinematics roughly means
to invert this mapping, i.e., given a desired target :math:`y^*` in task
space, find a joint vector :math:`q` such that :math:`\phi(q) = y^*`. As
often :math:`n>d`, the inversion is under-specified (leading to what is
called “redundancy”). But just as the pseudo-inverse of linear
transformation addresses this, we can generalize this to a non-linear
:math:`\phi` – namely in an optimality formulation.

Given :math:`\phi` and a target :math:`y^*`, a good option is to define
**inverse kinematics** as the non-linear mathematical program (NLP)

.. math::

   \begin{aligned}
   \label{eqIKNLP}
   q^* = \operatorname*{argmin}_q f(q) {~~\text{s.t.}~~}\phi(q) = y^* ~.\end{aligned}

\ The cost term :math:`f(q)` is called *regularization* and indicates a
preference among all solutions that satisfy :math:`\phi(q) = y`. One
might typically choose it as a squared distance
:math:`f(q) = {|\!|q-q_0|\!|}^2_W` to some “default” :math:`q_0`, which
could be the homing state of a robot or its current state.

In practice, I recommend always using a proper NLP solver to solve
inverse kinematics. As discussing optimization is beyond this script we
are here already done with describing inverse kinematics! It is “noting
more” than defining a constraint problem of the sort ([eqIKNLP]) and
passing it to a solver. In the coding part below I will discuss the
richness in options to define such constraint problems with our
differentiable features.

Only for educational purpose we will also derive the classical
pseudo-inverse Jacobian solution to IK below.

Building an NLP from features
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Eq. ([eqIKNLP]) describes IK as an NLP. Appendix [secNLP] provides a
technical reference of how we define NLPs mathematically and in code. We
summarize this here to enable us defining IK problems in the next coding
example. Essentially, we specify an NLP by *adding objectives*, i.e.,
adding entries to the total feature function :math:`\phi(x)` and
specifying the objective type:

.. math::

   \begin{aligned}
   \phi(x) = { \left(\begin{array}{c}f_1(x) \\ r_1(x) \\ h_1(x) \\ g_1(x) \\ h_2(x) \\ \vdots\end{array}\right) }
   {~,\quad}\rho = { \left(\begin{array}{c}{\texttt{f}}\\ {\texttt{sos}}\\ {\texttt{eq}}\\ {\texttt{ineq}}\\ {\texttt{eq}}\\ \vdots\end{array}\right) } ~.\end{aligned}

\ The indicator vector :math:`\rho` informs the solver which components
of :math:`\phi` have to be treated as cost, sos, eq, or ineq. The
entries “:math:`f_1`, :math:`r_1`,..” are any features defined in the
same convention as above. This defines an NLP of the form

.. math::

   \begin{aligned}
   \min_{b_l\le x \le b_u}~ {{{\bf 1}}}^{{\!\top\!}}\phi_{\texttt{f}}(x) + \phi_{\texttt{sos}}(x)^{{\!\top\!}}\phi_{\texttt{sos}}(x)
     {~~\text{s.t.}~~}\phi_{\texttt{ineq}}(x) \le 0,~ \phi_{\texttt{eq}}(x) = 0 ~,\end{aligned}

 where :math:`\phi_{\texttt{sos}}` is the subsets of ``sos``-features,
etc.

Let’s use a more interesting scene configuration to demonstrate IK. This
is really a core exercise, as it opens up the space of defining
kinematic constraint problems.

::

    import sys, os
    sys.path.append(os.path.expanduser('~/git/rai-python/build'))
    import libry as ry

    C = ry.Config()
    C.addFile(ry.raiPath('../rai-robotModels/scenarios/pandasTable.g'))
    C.view()

    C.addFrame('boxR','table') \
      .setRelativePosition([.15,0,.1]) \
      .setShape(ry.ST.ssBox, size=[.1,.1,.1,.02]) \
      .setColor([1,1,0])
    C.addFrame('boxL','table') \
      .setRelativePosition([-.15,0,.1]) \
      .setShape(ry.ST.ssBox, size=[.1,.1,.1,.02]) \
      .setColor([1,.5,0])
    C.view()

So far, we just created a new scene, with a yellow and orange box. Now
let’s define an NLP and solve it: [pgIK]

::

    komo = ry.KOMO()
    komo.setConfig(C, True)
    komo.setTiming(1., 1, 5., 1)
    komo.addControlObjective([], 0, 1e-1)
    komo.addObjective([], ry.FS.accumulatedCollisions, [], ry.OT.eq);
    komo.addObjective([], ry.FS.jointLimits, [], ry.OT.ineq);
    komo.addObjective([], ry.FS.positionDiff, ['r_gripper', 'boxL'], ry.OT.eq, [1e1]);
    komo.addObjective([], ry.FS.positionDiff, ['l_gripper', 'boxR'], ry.OT.eq, [1e1]);

    ret = ry.NLP_Solver() \
      .setProblem(komo.nlp()) \
      .setOptions( stopTolerance=1e-2 ) \
      .solve()
    print(ret)

    komo.view(False, "waypoint solution")

In the first block, we define a KOMO object, which is nothing but an NLP
description over configurations. The ``setConfig`` and ``setTiming``
calls state that we’re optimizing only over a single configuration, as
always in Inverse Kinematics. We’ll later explain how to optimize over
sequences of configurations.

The ``add`` methods add objectives (=cost terms, or eq, or ineq
constraints) to the NLP description. The ``ControlObjective`` is a
small-weighted regularization :math:`{|\!|q-q_0|\!|}^2` to optimize for
IK solutions close to the starting configuration. The others add
features that define equality (``ry.OT.eq``) or inequality
(``ry.OT.ineq``) constraints in the NLP.

This *language* of adding objectives to an NLP description is at the
core of the robotics library we use. Here it is used to define an
Inverse Kinematics problem. Later we can use it to define path
optimization problems, as well as MPC (reactive control) problems.

``NLP_Solver`` is a generic NLP solver (by default using an Augmented
Lagrangian method) that we introduce in the Optimization Algorithms
Lecture. The ``ret`` tells us how many steps (``evals``) the solver
needed, and what the costs and constraint errors at convergence are.

The ``komo`` display shows both, the initial configuration and the
solved configuration overlayed. With the following, we can read out the
optimal joint vector:

::

    q = komo.getPath()
    print(type(q), len(q))

    C.setJointState(q[0])
    C.view()

Classical Derivation of Pseudo-Inverse Jacobian Solution
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

I strongly recommend using an NLP solver and general constraint and cost
formulations to tackle IK problems – and you can skip over this section.
However, for completeness I provide here also the basic derivation of
classical pseudo-inverse Jacobian solutions.

Pseudo-inverse Jacobian.
^^^^^^^^^^^^^^^^^^^^^^^^

We first simplify the problem to minimize

.. math::

   \begin{aligned}
   \label{eqSoft}
   f(q) = {|\!|\phi(q) - y^*|\!|}^2_C + {|\!|q-q_0|\!|}^2_W ~.\end{aligned}

\ Instead of exactly ensuring :math:`\phi(q) = y^*`, this only minimizes
a penalty :math:`{|\!|\phi(q) - y^*|\!|}^2_C`. Here :math:`C` is the
norm’s metric, i.e., :math:`{|\!|v|\!|}^2_C = v^{{\!\top\!}}C v`, but
you may instead simply assume :math:`C` is a scalar. For finite
:math:`C` and :math:`W` this approximate formulation might be
undesirable. But later we will actually be able to investigate the limit
:math:`C\to\infty`.

Since this problem is a least squares problem, the canonical approach is
Gauss-Newton. The gradient, approximate Hessian, and Gauss-Newton step
are

.. math::

   \begin{aligned}
   {\frac{{\partial}}{{\partial}q}} f(q)
   &= 2 (\phi(q)-y^*)^{{\!\top\!}}C J + 2 (q-q_0)^{{\!\top\!}}W = {{\nabla_{\!\!f}}}(q)^{{\!\top\!}}\\
   {{\nabla_{\!\!f}^2}}(q)
   &\approx 2 (J^{{\!\top\!}}C J + W) \\
   {\delta}(q)
   &= - [{{\nabla_{\!\!f}^2}}(q)]^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}{{\nabla_{\!\!f}}}(q) = (J^{{\!\top\!}}C J + W)^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}[J^{{\!\top\!}}C (\phi(q)-y^*) + W (q-q_0) ]\end{aligned}

\ With some identities, this can be rewritten as

.. math::

   \begin{aligned}
   {\delta}(q)
   &= J^\sharp (y^* - \phi(q)) + (I - J^\sharp J)~ (q_0 - q) \label{eqIK} \\
   J^\sharp
   &= (J^{{\!\top\!}}C J + W)^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}J^{{\!\top\!}}C = W^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}J^{{\!\top\!}}(J W^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}J^{{\!\top\!}}+ C^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}})^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}\text{(Woodbury identity)}\end{aligned}

The matrix :math:`J^\sharp` is also called (regularized) pseudo-inverse
of :math:`J`. In its second form (RHS of Woodbury), we can take the hard
limit :math:`C\to\infty`, where
:math:`J^\sharp \to W^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}J^{{\!\top\!}}(J W^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}J^{{\!\top\!}})^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}`
or, for :math:`W={{\rm\bf I}}`,
:math:`J^\sharp \to J^{{\!\top\!}}(J J^{{\!\top\!}})^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}`.

Eq. ([eqIK]) says that, to jump to the (approx.) Gauss-Newton optimum,
we should make a step :math:`{\delta}` in joint space proportional to
the error :math:`(y^*-\phi(q))` in task space, and (optionally) combined
with a homing step towards :math:`q_0` projected to the task null space
via the projection :math:`(I - J^\sharp J)`.

Performing a single step :math:`{\delta}` is approximate due to the
non-linearity of :math:`\phi`. To solve inverse kinematics exactly we
have to iterate Gauss-Newton steps. If lucky, we can use full stepsizes
(:math:`{\alpha}= 1` in the speak of line search) and iterate
:math:`q_{k{{{{\hspace{-0.0pt}\textrm{+}\hspace{-0.5pt}}}1}}} \gets q_k + {\delta}(q_k)`
until convergence, and will have an exact IK solution. If :math:`\phi`
is very non-linear, we may have to do line searches along the step
directions to ensure convergence. If :math:`\phi` is non-convex, we may
converge to a local optimum that depends on the initialization.

On the fly IK.
^^^^^^^^^^^^^^

Inverse kinematics is sometimes being used to generate robot motion on
the fly. In a sense, rather than letting an optimization algorithm find
an IK solution and then start moving the robot to it (we we’ll do it
below), you let the robot directly move (generate a smooth path) towards
an IK solution. This is heuristic, and I eventually don’t recommend it.
But it’s standard practice, so let’s mention it:

Let the robot be in state :math:`q`, and we have a task space target
:math:`y^*`. We may compute a desired robot motion

.. math::

   \begin{aligned}
   \dot q = {\alpha}{\Big[}J^\sharp (y^* - \phi(q)) + (I - J^\sharp J) (q_0 - q) {\Big]}~.\end{aligned}

\ In a sense, this mimics performing (integrating over time)
infinitesimal Gauss-Newton steps towards the IK solution. Often the
regularization :math:`(I - J^\sharp J) (q_0 - q)` is also dropped, which
is the same as saying :math:`q_0 = q`, i.e., you always set the homing
state :math:`q_0` to be the current state :math:`q`, adapting it on the
fly. Doing this, you will loose a precise definition of where you’ll
eventually converge to – and sometimes this leads to undesired *drift in
nullspace*. All not recommended.

Singularity.
^^^^^^^^^^^^

The limit :math:`C\to\infty` mentioned above is only robust when
:math:`\det (J
J^{{\!\top\!}}) > 0`, or equivalently, when :math:`J` has full rank
(namely rank :math:`d`). :math:`J` is called singular otherwise, and the
pseudo inverse :math:`J^\sharp` is ill-defined.

Intuitively this means that, in state :math:`q`, certain task space
directions cannot be generated, i.e., no motion in these task space
directions is possible. A stretched arm that cannot extend further is a
typical example.

In the original NLP formulation, this corresponds to the case where
:math:`\phi(q) = y^*` is simply infeasible, and a proper NLP-solver
should return this information.

The soft problem formulation ([eqSoft]), where :math:`C` is finite (not
:math:`\infty`) is one way to address a singularity: For finite
:math:`C`, :math:`J^\sharp` is well defined and defines steps towards a
optimal solution of the trade-off problem ([eqSoft]). This is also
called **regularized IK** or **singularity-robust IK**. But it only
leads to an approximate IK solution.

Placeholder
===========

To come:

Spline Motion and Control Levels

Motion Planning: Path/Trajectory Optimization and Finding

Dynamics

Simulation

Control Theory & MPC

skipping here: Mobile Robotics slides

3D Transformations, Rotations, Quaternions
==========================================

Rotations
---------

There are many ways to represent rotations in :math:`SO(3)`. We restrict
ourselves to three basic ones: rotation matrix, rotation vector, and
quaternion. The rotation vector is also the most natural representation
for a “rotation velocity” (angular velocities). Euler angles or
raw-pitch-roll are an alternative, but they have singularities and I
don’t recommend using them in practice.

A rotation matrix
    is a matrix :math:`R\in{{\mathbb{R}}}^{3\times3}` which is
    orthonormal (columns and rows are orthogonal unit vectors, implying
    determinant 1). While a :math:`3\times3` matrix has 9 degrees of
    freedom (DoFs), the constraint of orthogonality and determinant 1
    constraints this: The set of rotation matrices has only 3 DoFs
    (:math:`\sim` the local Lie algebra is 3-dim).

    The application of :math:`R` on a vector :math:`x` is simply the
    matrix-vector product :math:`R x`.

    Concatenation of two rotations :math:`R_1` and :math:`R_2` is the
    normal matrix-matrix product :math:`R_1 R_2`.

    Inversion is the transpose,
    :math:`R^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}= R^{{\!\top\!}}`.

A rotation vector
    is an unconstrained vector :math:`w\in{{\mathbb{R}}}^3`. The
    vector’s direction :math:`\underline w = \frac{w}{|w|}` determines
    the rotation axis, the vector’s length :math:`|w|={\theta}`
    determines the rotation angle (in radians, using the right thumb
    convention).

    The application of a rotation described by
    :math:`w\in{{\mathbb{R}}}^3` on a vector
    :math:`x\in{{\mathbb{R}}}^3` is given as (Rodrigues’ formula)

    .. math::

       \begin{aligned}
       w \cdot x
        &= \cos{\theta}~ x
         + \sin{\theta}~ (\underline w\times x)
         + (1-\cos{\theta})~ \underline w(\underline w^{{\!\top\!}}x)\end{aligned}

    \ where :math:`{\theta}=|w|` is the rotation angle and
    :math:`\underline w=w/{\theta}` the unit length rotation axis.

    The inverse rotation is described by the negative of the rotation
    vector.

    Concatenation is non-trivial in this representation and we don’t
    discuss it here. In practice, a rotation vector is first converted
    to a rotation matrix or quaternion.

    Conversion to a matrix: For every vector
    :math:`w\in{{\mathbb{R}}}^3` we define its skew symmetric matrix as

    .. math::

       \begin{aligned}
       \hat w = \text{skew}(w) = { \left(\begin{array}{ccc}0 & -w_3 & w_2 \\ w_3 & 0 & -w_1 \\-w_2 & w_1 & 0\end{array}\right) } ~.\end{aligned}

    \ Note that such skew-symmetric matrices are related to the cross
    product: :math:`w \times v = \hat w~ v`, where the cross product is
    rewritten as a matrix product. The rotation matrix :math:`R(w)` that
    corresponds to a given rotation vector :math:`w` is:

    .. math::

       \begin{aligned}
       \label{eqRodriguez}
       R(w)
        &= \exp(\hat w) \\
        &= \cos{\theta}~ I + \sin{\theta}~ \hat w/{\theta}+ (1-\cos{\theta})~ w w^{{\!\top\!}}/{\theta}^2\end{aligned}

    \ The :math:`\exp` function is called exponential map (generating a
    group element (=rotation matrix) via an element of the Lie algebra
    (=skew matrix)). The other equation is called Rodrigues’ equation:
    the first term is a diagonal matrix (:math:`I` is the 3D identity
    matrix), the second terms the skew symmetric part, the last term the
    symmetric part (:math:`w
    w^{{\!\top\!}}` is also called outer product).

Angular velocity & derivative of a rotation matrix:
    We represent angular velocities by a vector
    :math:`w\in{{\mathbb{R}}}^3`, the direction :math:`\underline w`
    determines the rotation axis, the length :math:`|w|` is the rotation
    velocity (in radians per second). When a body’s orientation at time
    :math:`t` is described by a rotation matrix :math:`R(t)` and the
    body’s angular velocity is :math:`w`, then

    .. math::

       \begin{aligned}
       \label{eqDotR}
       \dot R(t) = \hat w~ R(t)~.\end{aligned}

    \ (That’s intuitive to see for a rotation about the :math:`x`-axis
    with velocity 1.) Some insights from this relation: Since
    :math:`R(t)` must always be a rotation matrix (fulfill orthogonality
    and determinant 1), its derivative :math:`\dot R(t)` must also
    fulfill certain constraints; in particular it can only live in a
    3-dimensional sub-space. It turns out that the derivative
    :math:`\dot R` of a rotation matrix :math:`R` must always be a skew
    symmetric matrix :math:`\hat w` times :math:`R` – anything else
    would be inconsistent with the constraints of orthogonality and
    determinant 1.

    Note also that, assuming :math:`R(0)=I`, the solution to the
    differential equation :math:`\dot R(t) = \hat w~ R(t)` can be
    written as :math:`R(t)=\exp(t \hat w)`, where here the exponential
    function notation is used to denote a more general so-called
    exponential map, as used in the context of Lie groups. It also
    follows that :math:`R(w)` from ([eqRodriguez]) is the rotation
    matrix you get when you rotate for 1 second with angular velocity
    described by :math:`w`.

Quaternion
    (I’m not describing the general definition, only the “quaternion to
    represent rotation” definition.) A quaternion is a unit length 4D
    vector :math:`r\in{{\mathbb{R}}}^4`; the first entry :math:`r_0` is
    related to the rotation angle :math:`{\theta}` via
    :math:`r_0=\cos({\theta}/2)`, the last three entries
    :math:`\bar r\equiv r_{1:3}` are related to the unit length rotation
    axis :math:`\underline w` via
    :math:`\bar r = \sin({\theta}/2)~ \underline w`.

    The inverse of a quaternion is given by negating :math:`\bar r`,
    :math:`r^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}=
    (r_0,-\bar r)` (or, alternatively, negating :math:`r_0`).

    The concatenation of two rotations :math:`r`, :math:`r'` is given as
    the quaternion product

    .. math::

       \begin{aligned}
       \label{eqQuat}
       r \circ r'
        = (r_0 r'_0 - \bar r^{{\!\top\!}}\bar r',~
           r_0 \bar r' + r'_0 \bar r + \bar r' \times \bar r)\end{aligned}

    The application of a rotation quaternion :math:`r` on a vector
    :math:`x` can be expressed by converting the vector first to the
    quaternion :math:`(0,x)`, then computing

    .. math::

       \begin{aligned}
       r \cdot x = (r \circ (0,x) \circ r^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}})_{1:3} ~,\end{aligned}

    \ I think a bit more efficient is to first convert the rotation
    quaternion :math:`r` to the equivalent rotation matrix :math:`R`, as
    given by

    .. math::

       \begin{aligned}
       R
        &= { \left(\begin{array}{ccc}
           1-r_{22}-r_{33} & r_{12}-r_{03} &    r_{13}+r_{02} \\
           r_{12}+r_{03} &   1-r_{11}-r_{33} &  r_{23}-r_{01} \\
           r_{13}-r_{02} &   r_{23}+r_{01} &    1-r_{11}-r_{22}
           \end{array}\right) } \\ & ~ r_{ij} := 2 r_i r_j ~.\end{aligned}

    \ (Note: In comparison to ([eqRodriguez]) this does not require to
    compute a :math:`\sin` or :math:`\cos`.) Inversely, the quaternion
    :math:`r` for a given matrix :math:`R` is

    .. math::

       \begin{aligned}
           r_0 &= {{\frac{1}{2}}}\sqrt{1+{{\rm tr}}R}\\
           r_3 &= (R_{21}-R_{12})/(4 r_0)\\
           r_2 &= (R_{13}-R_{31})/(4 r_0)\\
           r_1 &= (R_{32}-R_{23})/(4 r_0) ~.\end{aligned}

Angular velocity :math:`\to` quaternion velocity
    Given an angular velocity :math:`w\in{{\mathbb{R}}}^3` and a current
    quaternion :math:`r(t)\in{{\mathbb{R}}}`, what is the time
    derivative :math:`\dot r(t)` (in analogy to Eq. ([eqDotR]))? For
    simplicity, let’s first assume :math:`|w|=1`. For a small time
    interval :math:`{\delta}`, :math:`w` generates a rotation vector
    :math:`{\delta}w`, which converts to a quaternion

    .. math::

       \begin{aligned}
       \Delta r = (\cos({\delta}/2), \sin({\delta}/2) w) ~.\end{aligned}

    That rotation is concatenated LHS to the original quaternion,

    .. math::

       \begin{aligned}
       r(t+{\delta})
        = \Delta r \circ r(t) ~.\end{aligned}

     Now, if we take the derivative w.r.t. \ :math:`{\delta}` and
    evaluate it at :math:`{\delta}=0`, all the :math:`\cos({\delta}/2)`
    terms become :math:`-\sin({\delta}/2)` and evaluate to zero, all the
    :math:`\sin({\delta}/2)` terms become :math:`\cos({\delta}/2)` and
    evaluate to one, and we have

    .. math::

       \begin{aligned}
       \label{eqQuatVel}
       \dot r(t)
       &= {{\frac{1}{2}}}( - w^{{\!\top\!}}\bar r,~  r_0 w + \bar r \times w )
        = {{\frac{1}{2}}}(0,w) \circ r(t)\end{aligned}

    \ Here :math:`(0,w)\in{{\mathbb{R}}}^4` is a four-vector; for
    :math:`|w|=1` it is a normalized quaternion. However, due to the
    linearity the equation holds for any :math:`w`.

Quaternion velocity :math:`\to` angular velocity
    The following is relevant when taking the derivative w.r.t. the
    quaternion parameters, e.g., for a ball joint represented as
    quaternion. Given :math:`\dot r`, we have

    .. math::

       \begin{aligned}
       \dot r \circ r^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}&= {{\frac{1}{2}}}(0,w) \circ r \circ r^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}= {{\frac{1}{2}}}(0,w) {~,\quad}w = 2~ [\dot r \circ r^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}]_{1:3}\end{aligned}

    \ which allows us to read off the angular velocity :math:`w` induced
    by a change of quaternion :math:`\dot r`. However, the RHS zero will
    hold true only iff :math:`\dot
    r` is orthogonal to :math:`r` (where
    :math:`\dot r^{{\!\top\!}}r = \dot r_0 r_0 + \dot {\bar
    r^{{\!\top\!}}} \bar r = 0`, see ). In case
    :math:`\dot r^{{\!\top\!}}r \not=0`, the change in length of the
    quaternion does not represent any angular velocity; in typical
    kinematics engines a non-unit length is ignored. Therefore one first
    orthogonalizes :math:`\dot
    r \gets \dot r - r(\dot r^{{\!\top\!}}r)`.

    As a special case of application, consider computing the partial
    derivative w.r.t. quaternion parameters, where :math:`\dot r` is the
    4D unit vectors :math:`e_0,..,e_3`. In this case, the
    orthogonalization becomes simply :math:`e_i \gets e_i - r r_i` and

    .. math::

       \begin{aligned}
       (e_i - r_i r) \circ r^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}&= e_i \circ r^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}- r_i (1,0,0,0) \\
       w_i
       &= 2~ [e_i \circ r^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}]_{1:3} ~,\end{aligned}

    \ where :math:`w_i` is the rotation vector implied by
    :math:`\dot r = e_i`. In case the original quaternion :math:`r`
    wasn’t normalized (which could be, if a standard optimization
    algorithm searches in the quaternion parameter space), then
    :math:`r` actually represents the normalized quaternion
    :math:`\bar r = \frac{1}{\sqrt{r^2}} r`, and (due to linearity of
    the above), the rotation vector implied by :math:`\dot r = e_i` is

    .. math::

       \begin{aligned}
       \label{eqQuatJac}
       w_i
       &= \frac{2}{\sqrt{r^2}}~ [e_i \circ r^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}]_{1:3} ~.\end{aligned}

    \ This defines a :math:`3\times 4` **quaternion Jacobian**
    :math:`J_{:i} = w_i` with 4 columns :math:`w_i`, so that
    :math:`w = J \dot r` is the angular velocity induced by a quaternion
    velocity :math:`\dot r` (accounting for all implicit
    normalizations).

Transformations
---------------

We consider two types of transformations here: either static
(translation+rotation), or dynamic
(translation+velocity+rotation+angular velocity). The first maps between
two static reference frames, the latter between moving reference frames,
e.g. between reference frames attached to moving rigid bodies.

Static transformations
~~~~~~~~~~~~~~~~~~~~~~

Concerning the static transformations, again there are different
representations:

A homogeneous matrix
    is a :math:`4\times 4`-matrix of the form

    .. math::

       \begin{aligned}
       T = { \left(\begin{array}{cc}R & t \\ 0 & 1\end{array}\right) }\end{aligned}

    \ where :math:`R` is a :math:`3\times 3`-matrix (rotation in our
    case) and :math:`t` a :math:`3`-vector (translation).

    In homogeneous coordinates, vectors :math:`x\in{{\mathbb{R}}}^3` are
    expanded to 4D vectors
    :math:`{ \left(\begin{array}{c}x\\1\end{array}\right) } \in {{\mathbb{R}}}^4`
    by appending a 1.

    Application of a transform :math:`T` on a vector
    :math:`x\in{{\mathbb{R}}}^3` is then given as the normal
    matrix-vector product

    .. math::

       \begin{aligned}
       x' = T \cdot x
        &= T~ { \left(\begin{array}{c}x \\ 1\end{array}\right) }
         = { \left(\begin{array}{cc}R & t \\ 0 & 1\end{array}\right) }~ { \left(\begin{array}{c}x \\ 1\end{array}\right) }
         = { \left(\begin{array}{c}Rx + t \\ 1\end{array}\right) } ~.\end{aligned}

    Concatenation is given by the ordinary 4-dim matrix-matrix product.

    The inverse transform is

    .. math::

       \begin{aligned}
       T^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}&= { \left(\begin{array}{cc}R & t \\ 0 & 1\end{array}\right) }^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}= { \left(\begin{array}{cc}R^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}& -R^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}t \\ 0 & 1\end{array}\right) }\end{aligned}

Translation and quaternion:
    A transformation can efficiently be stored as a pair :math:`(t,r)`
    of a translation vector :math:`t` and a rotation quaternion
    :math:`r`. Analogous to the above, the application of :math:`(t,r)`
    on a vector :math:`x` is :math:`x' = t + r\cdot x`; the inverse is
    :math:`(t,r)^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}= (-r^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}\cdot t, r^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}})`;
    the concatenation is
    :math:`(t_1,r_1) \circ (t_2,r_2) = (t_1 + r_1\cdot t_2, r_1 \circ r_2)`.

Dynamic transformations
~~~~~~~~~~~~~~~~~~~~~~~

Just as static transformations map between (static) coordinate frames,
dynamic transformations map between moving (inertial) frames which are,
e.g., attached to moving bodies. A dynamic transformation is described
by a tuple :math:`(t,r,v,w)` with translation :math:`t`, rotation
:math:`r`, velocity :math:`v` and angular velocity :math:`w`. Under a
dynamic transform :math:`(t,r,v,w)` a position and velocity
:math:`(x,\dot x)` maps to a new position and velocity
:math:`(x',\dot x')` given as

.. math::

   \begin{aligned}
   & x'=t + r\cdot x \\
   & \dot x' = v + w \times (r\cdot x)+ r\cdot\dot x\end{aligned}

\ (the second term is the additional linear velocity of :math:`\dot x'`
arising from the angular velocity :math:`w` of the dynamic transform).
The concatenation
:math:`(t,r,v,w) = (t_1,r_1,v_1,w_1) \circ (t_2,r_2,v_2,w_2)` of two
dynamic transforms is given as

.. math::

   \begin{aligned}
   & t = t_1 + r_1 \cdot t_2 \\
   & v = v_1 + w_1 \times (r_1 \cdot t_2) + r_1 \cdot v_2 \\
   & r = r_1 \circ r_2 \\
   & w = w_1 + r_1 \cdot w_2\end{aligned}

\ For completeness, the footnote [1]_ also describes how accelerations
transform, including the case when the transform itself is accelerating.
The inverse
:math:`(t',r',v',w') = (t,r,v,w)^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}`
of a dynamic transform is given as

.. math::

   \begin{aligned}
   & t' = -r^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}\cdot t \\
   & r' =  r^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}\\
   & v' =  r^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}\cdot (w \times t - v) \\
   & w' = -r^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}\cdot w\end{aligned}

Sequences of transformations
    by :math:`T_{A\to
    B}` we denote the transformation from frame :math:`A` to frame
    :math:`B`. The frames :math:`A` and :math:`B` can be thought of
    coordinate frames (tuples of an offset (in an affine space) and
    three local orthonormal basis vectors) attached to two bodies
    :math:`A` and :math:`B`. It holds

    .. math::

       \begin{aligned}
       T_{A\to C} = T_{A\to B} \circ T_{B\to C}\end{aligned}

    \ where :math:`\circ` is the concatenation described above. Let
    :math:`p` be a point (rigorously, in the affine space). We write
    :math:`p^A` for the coordinates of point :math:`p` relative to frame
    :math:`A`; and :math:`p^B` for the coordinates of point :math:`p`
    relative to frame :math:`B`. It holds

    .. math::

       \begin{aligned}
       p^A = T_{A\to B}~ p^B ~.\end{aligned}

A note on affine coordinate frames
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Instead of the notation :math:`T_{A\to B}`, other text books often use
notations such as :math:`T_{AB}` or :math:`T^A_B`. A common question
regarding notation :math:`T_{A\to B}` is the following:

    *The notation :math:`T_{A\to B}` is confusing, since it transforms
    coordinates from frame :math:`B` to frame :math:`A`. Why not the
    other way around?*

I think the notation :math:`T_{A\to B}` is intuitive for the following
reasons. The core is to understand that a transformation can be thought
of in two ways: as a transformation of the *coordinate frame itself*,
and as transformation of the *coordinates relative to a coordinate
frame*. I’ll first give a non-formal explanation and later more formal
definitions of affine frames and their transformation.

Think of :math:`T_{W\to B}` as translating and rotating a real rigid
body: First, the body is located at the world origin; then the body is
moved by a translation :math:`t`; then the body is rotated (around its
own center) as described by :math:`R`. In that sense,
:math:`T_{W\to B} = { \left(\begin{array}{cc}R & t \\ 0
& 1\end{array}\right) }` describes the “forward” transformation of the
body. Consider that a coordinate frame :math:`B` is attached to the
rigid body and a frame :math:`W` to the world origin. Given a point
:math:`p` in the world, we can express its coordinates relative to the
world, :math:`p^W`, or relative to the body :math:`p^B`. You can
convince yourself with simple examples that
:math:`p^W = T_{W\to B}~ p^B`, that is, :math:`T_{W\to B}` *also*
describes the “backward” transformation of body-relative-coordinates to
world-relative-coordinates.

Formally: Let :math:`(A,V)` be an affine space. A coordinate frame is a
tuple :math:`(o,\boldsymbol e_1,..,\boldsymbol e_n)` of an origin
:math:`o \in A` and basis vectors :math:`\boldsymbol e_i \in V`. Given a
point :math:`p\in A`, its coordinates :math:`p_{1:n}` w.r.t. a
coordinate frame :math:`(o,\boldsymbol e_1,..,\boldsymbol e_n)` are
given implicitly via

.. math::

   \begin{aligned}
   p = o + \sum\nolimits_i p_i \boldsymbol e_i ~.\end{aligned}

\ A transformation :math:`T_{W\to B}` is a (“forward”) transformation of
the coordinate frame itself:

.. math::

   \begin{aligned}
   (o^B,\boldsymbol e^B_1,..,\boldsymbol e^B_n)
    &= (o^W + t, R\boldsymbol e^W_1,..,R\boldsymbol e^W_n)\end{aligned}

\ where :math:`t\in V` is the affine translation in :math:`A` and
:math:`R` the rotation in :math:`V`. Note that the coordinates
:math:`(\boldsymbol e^B_i)^W_{1:n}` of a basis vector
:math:`\boldsymbol e^B_i` relative to frame :math:`W` are the columns of
:math:`R`:

.. math::

   \begin{aligned}
   \boldsymbol e^B_i
    &= \sum_j (\boldsymbol e^B_i)^W_j \boldsymbol e^W_j
     = \sum_j R_{ji} \boldsymbol e^W_j\end{aligned}

\ Given this transformation of the coordinate frame itself, the
coordinates transform as follows:

.. math::

   \begin{aligned}
   p &= o^W + \sum_i p^W_i~ \boldsymbol e^W_i \\
   p &= o^B + \sum_i p^B_i~ \boldsymbol e^B_i \\
     &= o^W + t + \sum_i p^B_i~ (R \boldsymbol e^W_i) \\
     &= o^W + \sum_i t^W_i~ e^W_i + \sum_j p^B_j~ (R \boldsymbol e^W_j) \\
     &= o^W + \sum_i t^W_i~ e^W_i + \sum_j p^B_j~ (\sum_i R_{ij}~ \boldsymbol e^W_i) \\
     &= o^W + \sum_i {\Big[}t^W_i + \sum_j R_{ij}~ p^B_j{\Big]}~ e^W_i \\
   {\Rightarrow}&~ p^W_i = t^W_i + \sum_j R_{ij}~ p^B_j ~.\end{aligned}

\ Another way to express this formally: :math:`T_{W\to B}` maps
*covariant* vectors (including “basis vectors”) forward, but
*contra-variant* vectors (including “coordinates”) backward.

RAI references
==============

NLP interface
-------------

A general non-linear mathematical program (NLP) is of the form

.. math::

   \begin{aligned}
   \min_{b_l\le x \le b_u}~ f(x) ~{~~\text{s.t.}~~}~ g(x)\le 0,~ h(x) = 0  ~,\end{aligned}

\ with :math:`x\in{{\mathbb{R}}}^n`,
:math:`f:~ {{\mathbb{R}}}^n \to {{\mathbb{R}}}`,
:math:`g:~ {{\mathbb{R}}}^n \to {{\mathbb{R}}}^{d_g}`,
:math:`h:~ {{\mathbb{R}}}^n \to {{\mathbb{R}}}^{d_h}`,
:math:`b_l,b_u\in{{\mathbb{R}}}^n`. However, we want to explicitly
account for **least squares** costs (sum-of-squares), so that we extend
the form to

.. math::

   \begin{aligned}
   \min_{b_l\le x \le b_u}~ f(x) + r(x)^{{\!\top\!}}r(x) ~{~~\text{s.t.}~~}~ g(x)\le 0,~ h(x) = 0  ~,\end{aligned}

\ with :math:`r:~ {{\mathbb{R}}}^n \to {{\mathbb{R}}}^{d_r}`. In
technical terms, the solver needs to be provided with:

the problem “signature”: dimension :math:`n`, dimensions
:math:`d_r, d_g, d_h`, bounds :math:`b_l, b_u \in {{\mathbb{R}}}^n`,

functions :math:`f, r, g, h`,   Jacobians for all,   Hessian for
:math:`f`,

typically also an initialization sampler :math:`x_0 \sim p(x)`, that
provides starting :math:`x_0`.

Instead of providing a solver with separate functions
:math:`f, r, g, h`, we instead provide only a single differentiable
**feature** function :math:`\phi: X \to {{\mathbb{R}}}^K`, which stacks
all :math:`f,r,g,h` components to a single vector,

.. math::

   \begin{aligned}
   \phi(x) = { \left(\begin{array}{c}f_1(x) \\ r_1(x) \\ h_1(x) \\ g_1(x) \\ h_2(x) \\ \vdots\end{array}\right) }
   {~,\quad}\rho = { \left(\begin{array}{c}{\texttt{f}}\\ {\texttt{sos}}\\ {\texttt{eq}}\\ {\texttt{ineq}}\\ {\texttt{eq}}\\ \vdots\end{array}\right) } ~,\end{aligned}

\ where the indicator vector :math:`\rho` informs the solver which
components of :math:`\phi` have to be treated as cost, sos, eq, or ineq.
(The order of stacking does not matter.) In this convention, the NLP
reads

.. math::

   \begin{aligned}
   \min_{b_l\le x \le b_u}~ {{{\bf 1}}}^{{\!\top\!}}\phi_{\texttt{f}}(x) + \phi_{\texttt{sos}}(x)^{{\!\top\!}}\phi_{\texttt{sos}}(x)
     {~~\text{s.t.}~~}\phi_{\texttt{ineq}}(x) \le 0,~ \phi_{\texttt{eq}}(x) = 0 ~,\end{aligned}

\ where :math:`\phi_{\texttt{sos}}` is the subsets of ``sos``-features,
etc. The solver needs to be provided with:

the problem “signature”: dimension :math:`n`, feature types
:math:`\rho`, bounds :math:`b_l, b_u \in {{\mathbb{R}}}^n`,

a single differentiable **feature** function
:math:`\phi: X \to {{\mathbb{R}}}^K`, with Jacobian functnio
:math:`J = {\partial}_x \phi(x)`,

and typically also an initialization sampler :math:`x_0 \sim p(x)`, that
provides starting :math:`x_0`.

In the rai code, an NLP is therefore declared as

::

      //signature
      uint dimension;  ObjectiveTypeA featureTypes;  arr bounds_lo, bounds_up;

      //essential method
      virtual void evaluate(arr& phi, arr& J, const arr& x);

      //optional
      virtual arr  getInitializationSample(const arr& previousOptima={});
      virtual void getFHessian(arr& H, const arr& x);

Kinematic Features
------------------

The code has several kinematic features
:math:`\phi: q \mapsto \phi(q)\in{{\mathbb{R}}}^D` pre-defined – see
Table [tabFeatures].

+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| FS                      | frames   | :math:`D`   | :math:`k`   | description                                                                        |
+=========================+==========+=============+=============+====================================================================================+
| position                | o1       | 3           |             | 3D position of o1 in world coordinates                                             |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| positionDiff            | o1,o2    | 3           |             | difference of 3D positions of o1 and o2 in world coordinates                       |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| positionRel             | o1,o2    | 3           |             | 3D position of o1 in o2 coordinates                                                |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| quaternion              | o1       | 4           |             | 4D quaternion of o1 in world coordinates [2]_                                      |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| quaternionDiff          | o1,o2    | 4           |             | ...                                                                                |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| quaternionRel           | o1,o2    | 4           |             | ...                                                                                |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| pose                    | o1       | 7           |             | 7D pose of o1 in world coordinates                                                 |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| poseDiff                | o1,o2    | 7           |             | ...                                                                                |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| poseRel                 | o1,o2    | 7           |             | ...                                                                                |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| vectorX                 | o1       | 3           |             | The x-axis of frame o1 rotated back to world coordinates                           |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| vectorXDiff             | o1,o2    | 3           |             | The difference of the above for two frames o1 and o2                               |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| vectorXRel              | o1,o2    | 3           |             | The x-axis of frame o1 rotated as to be seend from the frame o2                    |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| vectorY...              |          |             |             | same as above                                                                      |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| scalarProductXX         | o1,o2    | 1           |             | The scalar product of the x-axis fo frame o1 with the x-axis of frame o2           |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| scalarProduct...        | o1,o2    |             |             | as above                                                                           |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| gazeAt                  | o1,o2    | 2           |             | The 2D projection of the origin of frame o2 onto the xy-plane of frame o1          |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| angularVel              | o1       | 3           | 1           | The angular velocity of frame o1 across two configurations                         |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| accumulatedCollisions   |          | 1           |             | The sum of collision penetrations; when negative/zero, nothing is colliding        |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| jointLimits             |          | 1           |             | The sum of joint limit penetrations; when negative/zero, all joint limits are ok   |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| distance                | o1,o1    | 1           |             | The NEGATIVE distance between convex meshes o1 and o2, positive for penetration    |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| qItself                 |          | :math:`n`   |             | The configuration joint vector                                                     |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| aboveBox                | o1,o2    | 4           |             | when all negative, o1 is above (inside support of) the box o2                      |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| insideBox               | o1,o2    | 6           |             | when all negative, o1 is inside the box o2                                         |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+
| standingAbove           |          |             |             | ?                                                                                  |
+-------------------------+----------+-------------+-------------+------------------------------------------------------------------------------------+

Table: [tabFeatures]Features pre-defined in rai.

Graph-Yaml Files
----------------

We use yaml-style files throughout. These are the file representation of
internal data structures such as dictionaries (anytype key-value maps)
used for parameter files or other structure data, but esp. also graphs.
The key extensions relative to yaml are:

-  An @Include@ node allows to hierarchically include files. This means
   that while each local file can be parsed with a standard yaml parser,
   an outer loop has to check for @Include@ nodes and coordinate loading
   sub-files.

-  As an implication of the above, we allow for a special @path@ type,
   as URLs embraced by ``<...>``. This becomes necessary as file values
   need to be interpreted relative to the path of the loading file. So
   when such a file is parsed we not only store the filename string, but
   also the path of the loading file to ensure we know its absolute
   path.

-  We also allow @Edit@ and @Delete@ tags, which allow us to
   edit/overwrite the value of previously defined nodes, as well as
   delete previously defined nodes.

-  Finally, the name of a node can include a list of parents: E.g. @A (B
   C): shape: box @ denotes a node with key @A@ that is a child of @B@
   and @C@. The semantics of this is that @A@ is a (directed) edge
   between B and C. This is analogous to a dot declaration @B -> C [
   shape=box ]@.

-  Note that all of the above is still yaml syntax, the outer parser
   only adds additional interpretation of @Include, Edit, Delete@ tags,
   @<..>@ values, and @(..)@ in names.

Within rai, .g-files are used to represent parameter files, robotic
configurations (:math:`\sim` URDF), logic, factor graphs, optimization
problems. The underlying data structure is used, e.g., as any-type
container, Graph, or auto-convertion to python dictionaries.

The following example of a .g-file might help:

::

    ## a trivial graph (all boolean-valued nodes)
    x            # a vertex: key=x, value=true, parents=none
    y            # another vertex: key=y, value=true, parents=none
    (x y)        # an edge: key=none, value=true, parents=x y
    (-1 -2)      # a hyperedge: key=none, value=true, parents=the previous edge and the y-node

    ## nodes with subgraphs as value
    A { color:blue }         # key=A, value=<Graph>, parents=none
    B { color:red, value:5 } # key=B, value=<Graph>, parents=none
    C(A,B) { width:2 }       # key=C, value=<Graph>, parents=A B
    hyperedge(A B C) : 5     # key=hyperedge, value=5, parents=A B C

    ## standard value types
    a:string      # MT::String (except for keywords 'true' and 'false' and 'Mod' and 'Include')
    b:"STRING"    # MT::String (does not require a ':')
    c:'file.txt'  # MT::FileToken (does not require a ':')
    d:-0.1234     # double
    e:[1 2 3 0.5] # MT::arr (does not require a ':')
    #f:(c d e)    # DEPRECATED!! MT::Array<*Node> (list of other nodes in the Graph)
    g!            # bool (default: true, !means false)
    h:true        # bool
    i:false       # bool
    j:{ a:0 }     # sub-Graph (special: does not require a ':')

    ## parsing: : {..} (..) , and \n are separators for parsing key-value-pairs
    b0:false b1, b2() b3    # 4 boolean nodes with keys 'b0', 'b1', 'b2', 'b3'
    k:{ a, b:0.2 x:"hallo"     # sub-Graph with 6 nodes
      y
      z():filename.org x }

    ## special Node Keys

    # editing: after reading all nodes, the Graph takes all Edit nodes, deletes the Edit tag, and calls a edit()
    # this example will modify/append the respective attributes of k
    Edit k { y:false, z:otherString, b:7, c:newAttribute }

    # including
    Include: 'example_include.g'   # first creates a normal FileToken node then opens and includes the file directly

    ## strange notations
    a()       # key=a, value=true, parents=none
    ()        # key=none, value=true, parents=none
    [1 2 3 4] # key=none, value=MT::arr, parents=none

Subgraphs may contain nodes that have parents from the containing graph,
or from other subgraphs of the containing graph. Some methods of the
``Graph`` class (to find nodes by key or type) allow to specify whether
also nodes in subgraphs or parentgraphs are to be searched. This
connectivity across (sub)-graphs e.g. allows to represent logic
knowledge bases.

yaml-style files to describe robot configurations
-------------------------------------------------

We use .g-files to represent robot/world configurations. .g-files
describe a general graph data structure as explained in :ref:‘refGraph‘.
But for robot configurations it is rather simple: Everly node describes
a frame, and is described by three things:

::

      <frame-name> ( <parent> ) { <attributes> }

where “<parent>“ needs to be a previously defined frame, or omitted, if
the frame is a root frame. The attributes defined properties of the
frame, such as its pose, shape, joint properties, etc.

Here is an example taken from the “test/Kin/kin“:

::

      stem { X:<t(0 0 .5)>, shape:capsule, mass:1, size:[1 .05] }
      
      joint1_pre (stem) { Q:<t(0 0 .5) d(90 1 0 0)> }
      joint1 (arm1) { joint:hingeX, Q:<d(-30 1 0 0)> }
      arm1 (joint1) { Q:<t(0 0 .15)>, shape:capsule, mass:1, size:[.3 .05] }
      
      arm2 { shape:capsule, mass:1, size:[.3 .05] }
      eff { shape:capsule, mass:1, size:[.3 .05] }
      
      joint2 (arm1 arm2) { joint:hingeX, A:<t(0 0 .15) d(0 0 0 1)>, Q:<d(-10 1 0 0)>, B:<t(0 0 .15) > }
      joint3 (arm2 eff ) { joint:hingeX, A:<t(0 0 .15) d(0 0 0 1)>, Q:<d(-10 1 0 0)>, B:<t(0 0 .15) > }
      
      target { X:<t(.0 .2 1.7)>, shape:sphere, mass:.001, size:[0 0 0 .02], color:[0 0 0] }

The first line defines a frame “stem“, which has absolute pose “<t(0 0
.5)>“ (pose specifications are described below). It also has a shape
attached, namely a capsule of length 1 and radius .05. And it has
inertial mass attached, namely with mass 1.

The 2nd to 4th line form a block of 3 new frames: the ``joint1_pre``
frame is child of step, which fixed relative transformation
``<t(0 0 .5) d(90 1 0 0)>`` (.5 meter up, 90 degress rotation about x).
The ``joint1`` frame is a child of ``joint1_pre``. But this frame is
special! It is a joint, which means that its relative transformation to
the parent is not fixed, but varies with joint dofs. Here, it is 1 joint
dofs describing a hinge joint about the parent’s x-axis. This joint is
here initialized to non-zero, namely to a relative transform
``<d(-30 1 0 0)>``. The ``arm1`` frame is then a child of ``joint1``,
with a relative transform ``<t(0 0 .15)>``, a capsule shape attached,
and mass.

This is a typical example for a chain of frames from one link, via a
joint, to the next. All robot configurations are just trees; and the
configuration file simply defines frames one-by-one, where each frame
may have 1 parent frame.

The next two lines define two more frames ``arm2`` and ``eff`` mass and
capsule shapes; but they’re yet all located at zero absolute pose. The
following two lines are actually a short hand notation to introduce a
joint frames between arm1 and arm2 (arm2 and arm3) in retrospect. The
``joint2`` declaration implicitly first defines a ``joint2_pre`` child
of arm1 with fixed relative transformation A; then the ``joint2`` chile
of ``joint2_pre`` with hinge joint and initial transformation Q; then
attaches the arm2 frame as its child with fixed relative transformation
B. So this is a typical short hand to specify a joint (more similarly to
how its done in URDF). But the generated underlying data structure is
just a tree of frames.

Editing using ``kinEdit``
~~~~~~~~~~~~~~~~~~~~~~~~~

Whenever working with .g-files, you should try to display them using the
``$RAI/bin/kinEdit`` command line tool. CMake automatically compiles it;
otherwise call ’make bin’ in ``$RAI`` to compile this. Then you can call
``kinEdit someFile.g`` on any model file. (In python, the equivalence is
to reload the configuration from file repeatedly.) Whenever ``kinEdit``
reads a file, it also outputs a file ``z.g`` and ``z.urdf`` of what it
read. Sometimes it is useful to look into this. It can also be used to
clean and prune kinematic structures.

But more than that, you can keep the display open when editing the file
in any text editor. Whenever you save the file the display will notice
it, reload the file, and display the updated model. This allows some
degree of continuous editing. You might sometimes have to hit enter in
the window to enforce reloading. The little tool tries to catch and
report on syntax errors and be robust, but it crashes on some syntax
errors and then needs to be restarted manually.

Import from URDF
~~~~~~~~~~~~~~~~

You can convert URDF files to .g-files using the ``rai/bin/urdf2rai.py``
script. However, the overall conversion is only partially automatic. The
resulting g-file encodes the full kinematic structure, but the mesh
files usually require manual fiddling. First, in the g-file, you have to
change the path to their location in the file system (removing the
’package’ part). Potentially that’s all you need. However, the rai code
calls various collision libraries that need clean and correct
(orientation, holes, etc) mesh files. Those that come with URDF files
are typically not clean and correct. I typically use meshlab (the
command line tool) to automatically clean and compress meshes into ply
files.

The best guide for the whole conversion pipeline is hubo/HOWTO.sh in the
rai-robotModels repository, which also describes mesh cleaning scripts.
We also managed to import a [full
kitchen](https://github.com/MarcToussaint/rai-robotModels/tree/master/bremenKitchen)
from unreal, where we first exported the description to JSON. There are
also some working examples to import ‘gltf‘.

Finally, the collada file format can represent trees of frames and
objects, which can be loaded. This can be augmented with just a little
extra information on joints to make this a properly articulated robot
world.

Notation to specify transformations
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Transformation can always be specified as 7-vectors
``Q:[p1 p2 p3 q0 q1 q2 q3]`` (position, quaternion), or also 3- or
4-vectors if you only want to set position or orientation. But this is
not always intuitive for human editing. Therefore, the bracket notation
``<...>`` allows for another notation, namely as a chaining of little
interpretable transformations, as in the old turtle language.

Specifically, you specify a transform by chaining:

::

      t(x y z)       # translation by (x,y,z)
      q(q0 q1 q2 q3) # rotation by a quaternion
      r(r x y z)     # rotation by `r` _radians_ around the axis (x,y,z)
      d(d x y z)     # rotation by `d` _degrees_ around the axis (x,y,z)
      E(r p y)       # rotation by roll-pitch-yaw Euler angles

Joint types
~~~~~~~~~~~

The ``libry.JT`` enum (in python; or rai::JointType in C++) lists all
available joint type. Currently these are:

::

      hingeX, hingeY, hingeZ, transX, transY, transZ, transXY, trans3, transXYPhi, universal, rigid, quatBall, phiTransXY, XBall, free, tau

A quatBall is a quaternion ball joint with 4 dofs (that supports all
differentiability and optimization); a free joint is a full 7 dof joint;
a rigid joint might seem redundant, but internally it sometimes markes a
break between separate objects (like an object sitting rigidly on a
table) rather than having multiple shapes attached to the same object.

The joint’s dofs can be initialized equivalently either with a ``q``
attribute (defining the dofs values), or a ``Q`` attribute (defining the
resulting relative transformation generated by the joint).

Cameras
=======

Image, Camera, & World Coordinates
----------------------------------

In this section, we use the following notation for coordinates of a 3D
point:

-  world coordinates :math:`X`,

-  camera coordinates :math:`x` (so that :math:`X = T x`, where
   :math:`T\equiv T_{W\to C}` is the camera position/orientation, also
   called **extrinsic parameter**),

-  image coordinates :math:`u=(u_x,u_y,u_d)`, with the pixel coordinates
   :math:`(u_x,u_y)` and depth coordinate :math:`u_d`, details as
   followed.

The pixel coordinates :math:`(u_x,u_y)` indicate where a point appears
on the image plane. The :math:`x`-axis always points to the right, but
there are two conventions for the :math:`y`-axis:

-  :math:`y`-up: The :math:`y`-axis points upward. This is consistent to
   how a diagram is typically drawn on paper: :math:`x`-axis right,
   :math:`y`-axis up. However, a consequence is that the :math:`z`-axis
   then points backward, i.e., pixels in front of the camera have
   negative depth :math:`u_d`.

-  :math:`y`-down: The :math:`y`-axis points down. This is consistent to
   how pixels are typically indexed in image data: counting rows from
   top to bottom. So when defining pixel coordinates :math:`(u_x,u_y)`
   literally to be pixel indices in image data, :math:`y`-down is the
   natural convention. A consequence is that the :math:`z`-axis points
   forward, i.e., pixels in front of the camera have a positive depth
   :math:`u_d`, which might also be more intuitive.

The transformation from camera coordinates :math:`x` to image
coordinates :math:`u` is involves perspective projection. For better
readability, let’s write – only for this equation –
:math:`x \equiv (x,y,z)`. Then the mapping is

.. math::

   \begin{aligned}
   \label{eqxtou}
   u = { \left(\begin{array}{c}u_x \\ u_y \\ u_d\end{array}\right) }
   &= { \left(\begin{array}{c}(f_x x + s y)/z + c_x\\ f_y y/z + c_y \\ z\end{array}\right) } ~.\end{aligned}

 Here, the five so-called **intrinsic parameters**
:math:`f_x,f_y,c_x,c_y,s` are the focal length :math:`f_x,f_y`, the
image center :math:`c_x,c_y`, and a image skew :math:`s` (which is
usually zero). E.g., for an image of height :math:`H` and width
:math:`W`, and vertical full view angle :math:`{\alpha}`, we typically
have an image center :math:`c_x \approx H/2, c_y \approx W/2` and a
focal length :math:`f_y
= \frac{H}{2 \tan({\alpha}/2)}`, e.g., for :math:`{\alpha}=90^\circ`,
:math:`f_y = H/2`. For a typical camera :math:`f_x \approx f_y`.

Inversely, if we have image coordinates :math:`u` and want to convert to
cartesian camera coordinates, we have (assuming :math:`s=0`)

.. math::

   \begin{aligned}
   x
   &= { \left(\begin{array}{c}(u_x - c_x) u_z / f_x\\ (u_y - c_y) u_z / f_y \\ u_z\end{array}\right) } ~.\end{aligned}

Homogeneous coordinates & Camera Matrix :math:`P`
-------------------------------------------------

First a brief definition: *A homogeneous coordinate
:math:`\boldsymbol x=(x_1,..,x_n,w)` is a (redundant) description of the
:math:`n`-dim point*

.. math:: {{\cal P}}(\boldsymbol x)= { \left(\begin{array}{c}x_1/w \\ \vdots \\ x_n/w\end{array}\right) } ~.

Note that two coordinates :math:`(x_1,..,x_n,w)` and
:math:`({\lambda}x_1,..,{\lambda}x_n,{\lambda}w)` are “equivalent” in
that they describe the same point. The operation :math:`{{\cal P}}` is
*non-linear* and called **perspective projection**. In this section, we
write homogeneous coordinates in bold :math:`\boldsymbol x`.

Back to our camera setting: Let :math:`\boldsymbol x` and
:math:`\boldsymbol X` be homogeneous camera and world coordinates of a
point (typically both have :math:`w=1` as last entry). Then the pose
transformation :math:`T` can be written as :math:`4\times` matrix such
that

.. math:: \boldsymbol x = T^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}\boldsymbol X ~.

Given camera coordinates :math:`x = ``(x,y,z)''`, we can write
([eqxtou])

.. math::

   \begin{aligned}
   \boldsymbol u
   &= K x
   = { \left(\begin{array}{c}f_x x + s y + c_x z\\ f_y y + c_y z \\ z\end{array}\right) } {~,\quad}K = { \left(\begin{array}{ccc}f_x & s & c_x \\ & f_y & c_y \\ & & 1 \end{array}\right) } {~,\quad}{{\cal P}}(\boldsymbol u)
   = { \left(\begin{array}{c}  (f_x x + s y)/z + c_x\\ f_y y/z + c_y \end{array}\right) } ~,\end{aligned}

\ where :math:`\boldsymbol u` are homogeneous *pixel* coordinates, and
:math:`{{\cal P}}(\boldsymbol u)` the actual pixel coordinates, which
would have to be augmented with :math:`z` again to get the :math:`u`
including depth coordinate.

The :math:`3\times 3` matrix :math:`K` includes the 5 general intrinsic
parameters. Writing the inverse transformation
:math:`T^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}` as a
:math:`3\times 4` matrix
:math:`{ \left(\begin{array}{cc}R^{{\!\top\!}}& -R^{{\!\top\!}}t\end{array}\right) }`
with rotation :math:`R` and translation :math:`t`, we can write the
relation between :math:`\boldsymbol u` and homogeneous world coordinates
:math:`\boldsymbol X` as

.. math::

   \begin{aligned}
   \boldsymbol u = P \boldsymbol X
   {~,\quad}\text{with~} P = { \left(\begin{array}{cc}K & 0\end{array}\right) }~ T^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}= { \left(\begin{array}{cc}K & 0\end{array}\right) }~ { \left(\begin{array}{cc}R^{{\!\top\!}}& -R^{{\!\top\!}}t \\ & 1\end{array}\right) } = { \left(\begin{array}{cc}KR^{{\!\top\!}}& -KR^{{\!\top\!}}t\end{array}\right) } ~,\end{aligned}

\ where :math:`P` is the :math:`3\times 4` **camera matrix**, which
subsumes 5 intrinsic and 6 extrinsic (3 rotation, 3 translation)
parameters. Except for absolute scaling (the 1 in the definition of
:math:`K`), this fully parameterizes a general affine transform.

Calibration as Estimating :math:`P,K,R,t` from Depth Data
---------------------------------------------------------

Assuming we have data of pairs :math:`(\boldsymbol u, \boldsymbol X)`,
we can use the basic equation :math:`\boldsymbol u = P \boldsymbol X` to
retrieve :math:`P` in closed from, and in a second step retrieve the
intrinsic and extrinsic camera parameters from :math:`P`. Note that here
we discuss the situation where we have the “right” :math:`\boldsymbol u`
in the data – and not only the pixel coordinates
:math:`{{\cal P}}(\boldsymbol u)`! This means that we assume we have
data entries :math:`\boldsymbol u = (u_x u_d, u_y u_d, u_d)` which
includes the true depth :math:`u_d`. So this method is only applicable
when we want to calibrate a depth camera.

Given data :math:`D = \{(\boldsymbol u_i, \boldsymbol X_i)\}_{i=1}^n`,
we want to minimize the squared error

.. math::

   \begin{aligned}
   \operatorname*{argmin}_P \sum_i (\boldsymbol u_i - P \boldsymbol X_i)^2 = [U - P X]^2 ~,\end{aligned}

\ where :math:`U` and :math:`X` are the stacked :math:`\boldsymbol u_i`
and :math:`\boldsymbol X_i`, respectively. The solution is
:math:`P = U^{{\!\top\!}}X (X^{{\!\top\!}}X)^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}`.
Comparing with the form of :math:`P` above, we can decompose it and
extract explicit :math:`K, R, t` using

.. math::

   \begin{aligned}
     (K,R^{{\!\top\!}}) &\gets \text{RQ-decomposition}(P_{1:3,:}) \\
     t &\gets -(K R^{{\!\top\!}})^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}P_{4,:}\end{aligned}

However, when defining
:math:`\bar u = (\boldsymbol u,1) = (u_x u_z, u_y u_z, u_z, 1)` (with
additional 1 agumented), we can also write the inverse linear relation
to the non-homogeneous world coordinate :math:`X`:

.. math::

   \begin{aligned}
   X  = P^+ \bar u{~,\quad}\text{with~} P^+ = { \left(\begin{array}{cc}R K^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}& t\end{array}\right) } \bar u~,\end{aligned}

\ Using data :math:`X` (:math:`3\times n`) and :math:`U`
(:math:`4\times n`) the optimum is
:math:`P^+ = X^{{\!\top\!}}U (U^{{\!\top\!}}U)^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}`.
We can decompose it using

.. math::

   \begin{aligned}
     t &\gets P^+_{3,:} \\
     (K,R^{{\!\top\!}}) &\gets \text{RQ-decomposition}( [P^+_{1:3,:}]^{{{{\hspace{-0.0pt}\textrm{-}\hspace{-0.5pt}}}1}}]\end{aligned}

Placeholder
===========

Shapes
------

Kinematics formally
-------------------

– Switching Kinematics?

.. [1]
   Transformation of accelerations:

   .. math::

      \begin{aligned}
      \dot v
       &= \dot v_1
            + \dot w_1 \times (r_1 \cdot t_2)
            + w_1 \times (w_1 \times (r_1 \cdot t_2))\\
            &\quad+ 2\, w_1 \times (r_1 \cdot v_2)
            + r_1 \cdot \dot v_2 \\
      \dot w
       &= \dot w_1 + w_1 \times (r_1 \cdot w_2) + r_1 \cdot \dot w_2\end{aligned}

   \ Used identities: for any vectors :math:`a,b,c` and rotation
   :math:`r`:

   .. math::

      \begin{aligned}
      &r \cdot (a \times b) = (r \cdot a) \times (r \cdot b)\\
      & a \times (b \times c) = b (a c) - c (ab) \\
      &{\partial}_t (r \cdot a) = w \times (r \cdot a) + r \cdot \dot a \\
      & {\partial}_t (w \times a) = \dot w \times t + w \times \dot a\end{aligned}

.. [2]
   There is ways to handle the invariance w.r.t. quaternion sign
   properly.
