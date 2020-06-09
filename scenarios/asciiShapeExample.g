# this is a mini example for having a shape that is the convex hull (plus sphere sweeping) of points given in an ascii file -- really the poor man`s way to design a shape
# blender would be a MUCH MUCH more evolved way to design shapes
# but for simple cases, why not directly edit them in the ascii file

# use 'kinEdit' while editing the .arr file to immediately get graphical feedback on the shape


world {}

myshape (world) {
    shape:ssCvx, # this states "sphere-swept convex"
    mesh:'asciiShapeExample.arr', # this specifies the core of the object
    size:[.01] # the sphere-sweeping radius
    color:[.8, .8, .2], # the color
    Q:<t(0 0 1)> # relative pose: translated by (0,0,1)
}
