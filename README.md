# MLR robotics course

This repo is based on RAI code, including its python bindings. See https://github.com/MarcToussaint/rai for a README of the RAI code.

## Quick Start

This assumes a standard Ubuntu 16.04 machine.

WE DIDN'T GET TO RUN THIS WITH ANACONDA PYTHON. I you have Anaconda
installed, please remove it from the PATH in .bashrc. The setup below will
install the standard Ubuntu python3 and jupyter notebook.

```
git clone https://github.com/MarcToussaint/robotics-course.git
cd robotics-course

git submodule init
git submodule update

make -j1 initUbuntuPackages  # calls sudo apt-get install; you can always interrupt
make -j4                     # builds libs and tests

source setupPython.sh

python3 -m pip install --upgrade pip
python3 -m pip install jupyter

jupyter-notebook docs/1-basics.ipynb 
```

## Tutorials

Only a few of the tutorials exist yet. Please see the also [docs/](docs/) path. The plan is:

1. [Basics:](docs/1-basics.ipynb) Configurations, Features & Jacobians
1. [IK:](docs/2-constraints.ipynb) Learn about the language to set optimization constraints, first with just Inverse Kinematics; grabbing results
1. [KOMO:](docs/3-KOMO.ipynb) Interface to KOMO, the motion optimization method; learn to set constraints
1. [LGP:](docs/4-LGP.ipynb) The low-level skeleton interface to solving LGP problems
1. [Contacts:](docs/8-contacts.ipynb) Access to various methods to compute detailed collision geometries or compute stable force/wrench configurations, all static
1. [Physx:](docs/9-physx.ipynb) Access to the Physx physical simulation engine
1. [Bullet:](docs/10-bullet.ipynb) Access to the Physx physical simulation engine

## Examples

Check the [examples/](examples/) path

## Older/messy docs

Just as a reference: https://github.com/MarcToussaint/rai-maintenance/tree/master/help

