Include: '../../scenarios/liftRing.g'

Edit gripper { contact:-2 }
Edit box { Q:<t(.2 0. .25) d(40 1 1 0)>, mass:.1 }
Edit stick { Q:<t(-.3 .6 1.1) d(90 1 0 0) d(20 1 1 0)> }
stickTip (stick) {shape:marker size: [.1] Q:<t(0 0 .3)>}

Edit panda_link0 (table) { joint: rigid Q:<t(.3 .8 .05) d(-90 0 0 1)>, motors}


### camera

camera(world){
    Q:<t(.0 .0 2.) d(90 0 0 1)>,
    shape:marker, size:[.1],
    focalLength:0.895, width:640, height:360, zRange:[.5 100]
}
