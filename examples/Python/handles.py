from openscad import *
c=cube([10,10,10])

print(translate(c.origin,[5,0,0]))
# translate the origin with an offset, so top_center sits on top of the cube
#c.top_center=translate(c.origin,[5,5,10])

# This one even points to the right side
#c.right_center=translate(roty(c.origin,90),10,5,5)

# The handles can be used with align
#cyl = cylinder(d=1,h=2)

# This placecs cyl onto the right side of the cube - of course rotated
#    obj       source handle  dest handle
#c |= cyl.align(c.right_center,cyl.origin)

c.show()

