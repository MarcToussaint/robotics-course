# kinematic graph
body stem { X=<T t(0 0 1)>  shape:capsule mass=.1 size=[0.1 0.1 2 .1] fixed }

body arm1 { shape:capsule mass=.1 size=[0.1 0.1 .4 .1] }
body arm2 { shape:capsule mass=.1 size=[0.1 0.1 .4 .1] }
body peg { shape:capsule mass=.1 size=[0.1 0.1 .6 .05] contact color=[0 .5 0] }

joint (stem arm1) { joint:hingeX A=<T t(0 0 1) d(90 1 0 0) > B=<T t(0 0 .2) >  m }
joint (arm1 arm2) { joint:hingeX A=<T t(0 0 0.2) > B=<T t(0 0 .2) >  m }
joint (arm2 peg) { joint:hingeX A=<T t(0 0 0.2) > B=<T t(0 0 .4) >  m }
