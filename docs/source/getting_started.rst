Getting Started
===============

Quick install
-------------

Install the ``robotic`` python package. On a standard Ubuntu, the
following should be sufficient:

::

   sudo apt install liblapack3 freeglut3 libglew-dev python3 python3-pip
   python3 -m pip install --user robotic numpy scipy

A standard test is

::

   python3 -c 'from robotic import ry; print("ry version:", ry.__version__, ry.compiled());'
   python3 -c 'from robotic import ry; ry.test.RndScene()'

Avanced info
------------

-  **Compiling for the real robot:** The ``robotic`` pip package does
   not have the real robot drivers (for the Panda robot) included, and
   therefore only supports simulation mode. However, the interface and
   code is designed so that everything directly transfers to the real
   robot: All you have to do is re-compile the python package on your
   machine, including the libfranka and librealsense drivers. This is
   done by installing the USE PDFLATEX! package.
-  **Sources for the wheel:** The pip package is created with the
   following repo: USE PDFLATEX!. The hardest part was to create a
   Docker that compiles ALL the many dependencies as static libraries in
   a CentOS - once that’s done, the botop lib can be compiled in a
   compatible manner, not depending on shared libs.
-  **Docstrings and tab completion for ry:** When you code in an IDE
   (e.g. VS code), you definitely want to have tab completion and
   docstrings for the methods. The ry module is compiled using pybind11
   from C++ code and natively lacks docstrings and tab completion.
   However, using pybind11-stubgen one can generate something like a
   *header file* for the whole python module, called ry.ipy. That file
   resides in ``~/.local/lib/python*/site-packages/robotic/ry.ipy`` next
   to the ``ry.py`` and is important to get tab completion working.
   Ensure that after installing ``robotic``, your IDE supports tab
   completion for ry classes and methods.

   If tab-autocomplete for jupyter does not work, try
   ``pip3 install   --user jedi==0.17.2``

Installing the tutorial notebooks
---------------------------------

::

   git clone https://github.com/MarcToussaint/rai-python.git
   cd rai-python/notebooks
   jupyter-notebook .

If you don’t have jupyter installed yet:

::

   pip3 install --user jupyter nbconvert matplotlib
