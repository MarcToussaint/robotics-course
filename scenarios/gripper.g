gripper{
    shape:ssBox, size:[.14 .04 .03 .01], color:[.9 .9 .9]
    X:<t(.05 .5 1.) d(-90 0 0 1) d(-45 1 0 0) >
    contact:-1
}

finger1 (gripper){
    shape:sphere, color:[.9 .9 .9], size:[.03]
    Q:<t(+.07 0 -.05)>
    contact:-1
}

finger2 (gripper){
    shape:sphere, color:[.9 .9 .9], size:[.03]
    Q:<t(-.07 0 -.05)>
    contact:-1
}

gripperCenter (gripper){
    shape:marker, size:[.03], color:[.9 .9 .9],
    Q:<t(0 0 -.05)>
}
