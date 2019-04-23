Include = '../../rai-robotModels/baxter/baxter.g'

frame camera (head) { shape:marker, Q:<d(-90 0 0 1) t(0 .1 -.2) d(-75 1 0 0)>, color:[.8 .8 0] size:[.2], focalLength:0.895, width:640, height:480, zRange:[.1 10] }
frame cameraBlock (camera) { shape:box size:[.2 .03 .06] color:[0 0 0] }
