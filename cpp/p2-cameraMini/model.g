Include = '../../rai-robotModels/baxter/baxter_new.g'

frame camera (head) { shape:marker,
#                      Q:<d(-90 0 0 1) t(-.05 .13 -.25) d(4 0 1 0) d(-67 1 0 0)>,
                      Q:<d(180 0 0 1) t(-.150 0.073 0.071) E(-2.598 -0.052 -1.485)>,
                      color:[.8 .8 0] size:[.2], focalLength:0.895, width:640, height:480, zRange:[.1 10] }

frame cameraBlock (camera) { shape:box size:[.2 .03 .06] color:[0 0 0] }
