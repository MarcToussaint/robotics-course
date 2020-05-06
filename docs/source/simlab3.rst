===================
 SimLab Exercise 3
===================

Note: Before new exercises, always update the repo::

  cd $HOME/git/robotics-course
  git pull
  git submodule update
  cd build
  make -j4


CODE CHANGES:
* The komo.optimize (in both, python and C++) does not get a Boolean argument anymore, only a double that specifies the initialization noise
* In python, playVideo now has a delay argument so slow down display (also a pathName argument to save video pics)
* The 'simplex types 4 4 not handled' bug is fixed
* The pandas now have named collision shapes panda_coll_[0..7] and panda_coll_hand, panda_coll_finger[1,2]
* (Can't see why vectorXRel calls NIY -- need to reproduce first.)
  
Note: The librai.so (which is linked by both, our C++ and python code) links
to the Ubuntu package libopencv-dev. However, python coders call
opencv directly using the pip-installed package. Hopefully this does
not lead to library version clashes.




OpenCV usage
============

The goal of this session is to enable you using OpenCV within your code.

The first example in @04-opencv@ subscribes to a webcam and starts a
little loop that displays the image using opencv. The webcam is only
to have more fun during coding -- later your code has to run on the
simulated camera. The second example in @04-opencv@ does our standard
simulation loop, but in each iteration grabs the simulated RGB and
depth image, displays them using opencv, converts them to a point
cloud, and displays this point cloud in your model configuration. So
this is your first example of bringing camera signals into your 3D
model world.

In this exercise you should add opencv code (directly operating on
@cv::Mat@) to these examples to perform some basic image processing.

There are many OpenCV tutorials:
* For python, see https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_tutorials.html
* In C++, see https://docs.opencv.org/3.2/doc/tutorials/tutorials.html (perhaps change to another OpenCV version)
Esp, see "Image Processing".


OpenCV tutorials
================

Ideally, implement these exercises within the webcam example, in the
online loop. If this is troublesome, first doing them on a static
image loaded from file is also ok.

Exercise 1:
-----------
* Filter the color of the image to find all pixels that are redish.
* Display the binary image that indicates red pixels.
* Fit contours to this binary image, which returns the segments.
* Display the contours by drawing them into the original RGB image

Exercise 2:
-----------
* Store the first image as background image. (Or average over several first images.)
* For every new image filter those pixels, that are significantly different to the background.
* Display the binary image that indicates change pixels.
* Fit contours to this binary image, which returns the segments.
* Display the contours by drawing them into the original RGB image
* Test the same pipeline, but first smoothing/blurring the image

Exercise 3 (Bonus):
-------------------

Exercise 2 is doing foreground/background segmentation based on the
RGB. Think about how one can do the same for depth. In particular,
assume that 'background' has always the largest pixel depth -- so when
the depth of a pixel is less than before, then it must be
foreground. Realize this within the simulation loop, where you have
access to depth.


