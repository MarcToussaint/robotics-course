# MLR robotics course

This repo is based on RAI code, including its python bindings. See https://github.com/MarcToussaint/rai for a README of the RAI code.

## Quick Start

This assumes a standard Ubuntu 16.04 machine.

WE DIDN'T GET TO RUN THIS WITH ANACONDA PYTHON. If you have Anaconda
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

1. [Basics:](docs/1-basics.ipynb) Configurations, Features & Jacobians




