Include = '../../rai-robotModels/baxter/baxter_new.g'

frame camera (head) { shape:box,
                      Q:<d(-90 0 0 1) t(-.08 .205 .115) d(26 1 0 0) d(-1 0 1 0) d(6 0 0 1)>,
                      size:[.2 .03 .06] color:[0 0 0]
                      focalLength:0.895, width:640, height:480, zRange:[.1 10] }

frame calibR (right_wrist){ shape:sphere size:[.02] color:[0 .5 0] Q:<t(0 .065 .138)> }
frame calibL (left_wrist){ shape:sphere size:[.02] color:[0 .5 0] Q:<t(0 -.065 .138)> }

frame volumeL { shape:ssBox size:[.4 .5 .5 .01] color:[.5 0 0 .2] X:<t(-.2 .6 1.1)>, noVisual }
frame volumeR { shape:ssBox size:[.4 .5 .5 .01] color:[.5 0 0 .2] X:<t(.2 .6 1.1)>, noVisual }
