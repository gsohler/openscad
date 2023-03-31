import libfive as lv

class Coord:
    def __init__(self):
        self.pos = lv.x(),lv.y(),lv.z()

    def trans(self, v):
        self.pos = self.pos[0]-v[0] ,self.pos[1]-v[1],self.pos[2]-v[2]

class Object:
    def sphere(c,r):
        return c.pos[0]*c.pos[0]+c.pos[1]*c.pos[1]+c.pos[2]*c.pos[2]-r*r


def lv_coord():
        return lv.x(),lv.y(),lv.z()

def lv_clamp(c,low, high):
    return lv.min(lv.max(c,low),high)

def lv_lerp(a,b,t):
    return  a*(1-t) + b*t

def lv_trans(c,v):
    return c[0]-v[0] ,c[1]-v[1],c[2]-v[2]

def lv_sphere(c,r):
    return c[0]*c[0]+c[1]*c[1]+c[2]*c[2]-r*r

def lv_box(c, box):
    return lv.max(lv.max(
        lv.max(c[0]-box[0],-c[0]),
        lv.max(c[1]-box[1],-c[1])
    ),  lv.max(c[2]-box[2],-c[2]))


def lv_union(a, b):
    return lv.min(a, b)

def lv_union_chamfer(a, b,r): # Credit: Doug Moen
    e = lv.max(r - lv.abs(a - b), 0);
    return lv.min( a, b) - e*0.5

def lv_union_smooth(a, b, r): #Credit: original algorithm in GLSL by Inigo Quilez

    h = lv_clamp (((b-a)/r+1)*0.5, 0, 1)
    return lv_lerp(b, a, h) + r*h*(h-1)

def lv_union_stairs(a,b,r,n): # Credit: original algorithm in GLSL by @paniq
    s = r/(n+1.0)
    u = b-r;
    return lv.min( lv.min(a,b), ( a+u+lv.abs( (-a+u+s)%(2*s) - s))*0.5)


def lv_intersection(a, b):
    return -lv_union(-a,-b) 

def lv_intersection_chamfer(a, b,r):
    return -lv_union_chamfer(-a,-b,r)

def lv_intersection_smooth(a, b,r):
    return -lv_union_smooth(-a,-b,r)

def lv_intersection_stairs(a, b,r,n):
    return -lv_union_stairs(-a,-b,r,n)

def lv_difference(a, b):
    return -lv_union(-a,b) 

def lv_difference_chamfer(a, b,r):
    return -lv_union_chamfer(-a,b,r)

def lv_difference_smooth(a, b,r):
    return -lv_union_smooth(-a,b,r)

def lv_difference_stairs(a, b,r,n):
    return -lv_union_stairs(-a,b,r,n)

         
