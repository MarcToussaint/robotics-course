============================
 SimLab Exercise 3 - OpenCV
============================

Note: Before new exercises, always update the repo::

  cd $HOME/git/robotics-course
  git pull
  git submodule update
  cd build
  make -j4



The goal of this session is to enable you to use OpenCV within your code.

The first example in ``04-opencv`` subscribes to a webcam and starts a
little loop that displays the image using opencv. The webcam is only
to have more fun during coding -- later your code has to run on the
simulated camera. The second example in ``04-opencv`` does our standard
simulation loop, but in each iteration grabs the simulated RGB and
depth image, displays them using opencv, converts them to a point
cloud, and displays this point cloud in your model configuration. So
this is your first example of bringing camera signals into your 3D
model world.

Here you can find OpenCV documentation:

* In C++, see https://docs.opencv.org/master/, in particular the *OpenCV Tutorials - Image Processing* section
* For python, see https://opencv-python-tutroals.readthedocs.io/en/latest/py_tutorials/py_tutorials.html

  
a) OpenCV - color-based tracking
================================

The goal of this exercise is to implement a 'red object tracker'. First develop it within the webcam example. Then test if you can also track the falling red object in the second example of ``04-opencv``. Specifically:

1. Implement a color filter to find all pixels that are redish.
2. Display the binary mask image that indicates red pixels.
3. Find contours to this binary image, which returns the segments.
4. Display the contours by drawing them into the original RGB image.

Then, when applying to the dropping red object in the simulated world:

5. Identify the "center of red" in camera coordinates, identify the
   "mean red depth", and combine this to yield the 3D position
   estimate. Create a simple sphere in your model world that moves
   with this tracking estimate.
   

b) OpenCV - background substraction
===================================

Working with you webcam:

1. Store the first image as background image. (Or average over several first images.)
2. For every new image filter those pixels, that are significantly different to the background.
3. Display the binary image that indicates change pixels.
4. Fit contours to this binary image, which returns the segments.
5. Display the contours by drawing them into the original RGB image
6. Test the same pipeline, but first smoothing/blurring the image
   
Bonus/for discussion: Think about how one can do the same for
depth. In particular, assume that the "background" has always the
largest pixel depth -- so when the depth of a pixel is less than
before, then it must be foreground. Realize this within the simulation
loop, where you have access to depth.


