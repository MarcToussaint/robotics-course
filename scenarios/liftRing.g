world {}

table (world){
    shape:ssBox, Q:<t(0 0. .6)>, size:[1. 2. .1 .02], color:[.3 .3 .3]
    fixed, contact, logical:{ }
    friction:.1
}

box (table){
    shape:ssBox, Q:<t(.2 0. .15)>, size:[.2 .2 .2 .02], color:[.6 .6 .6]
    mass:.1
    joint:rigid
    friction:.1
}

ring (box){
    Q:<t(0 0 .2)>
}

ring0(ring){ shape:capsule, Q:<t(0 0 -.1)>, size:[.05 .02], color:[.6 .6 .6]
    logical: {object} }
ring1(ring){ shape:capsule, Q:<t(.07 0 0)>, size:[.1 .02], color:[.6 .6 .6]
    logical: {object} }
ring2(ring){ shape:capsule, Q:<t(-.07 0 0)>, size:[.1 .02], color:[.6 .6 .6]
    logical: {object} }
ring3(ring){ shape:capsule, Q:<t(0 0 -.05) d(90 0 1 0)>, size:[.1 .02], color:[.6 .6 .6]
    logical: {object} }
ring4(ring){ shape:capsule, Q:<t(0 0 .05) d(90 0 1 0)>, size:[.1 .02], color:[.6 .6 .6]
    logical: {object}, contact:-1 }

stick (table){
    shape:capsule, Q:<t(-.3 .6 .1) d(90 1 0 0)>, size:[.6 .02], color:[.6 .6 .6]
    joint:rigid
    mass:1
}

target (table){
    shape:ssBox, Q:<t(-.2 0 .2)>, size:[.3 .3 .05 .02], color:[.3 .6 .3 1]
}

Include '../rai-robotModels/scenarios/panda_fixGripper.g'
joint (table panda_base){ joint:rigid Q:<t(.3 .8 .05) d(-90 0 0 1)> }
