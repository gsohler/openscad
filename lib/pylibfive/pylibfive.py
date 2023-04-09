import libfive as lv

#http://www.gradientspace.com/tutorials/category/g3sharp


def lv_coord():
        return lv.x(),lv.y(),lv.z()

def lv_clamp(c,low, high):
    return lv.min(lv.max(c,low),high)

def lv_lerp(a,b,t):
    return  a*(1-t) + b*t

def lv_veclerp(a,b,t):
    return  lv_lerp(a[0],b[0],t), lv_lerp(a[1],b[1],t), lv_lerp(a[2],b[2],t)

def lv_dot(a,b):
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]

def lv_trans(c,v):
    return c[0]-v[0] ,c[1]-v[1],c[2]-v[2]

def lv_length(c):
    return lv.sqrt(c[0]*c[0]+c[1]*c[1]+c[2]*c[2])

def lv_length1(x,y,z):
    return lv.sqrt(x*x+y*y+z*z)



def lv_sphere(c,r):
    return lv_length(c)-r 

def lv_box(c, box):
	q= lv.abs(c[0])-box[0], lv.abs(c[1])-box[1], lv.abs(c[2])-box[2]
	return lv_length1( lv.max(q[0],0), lv.max(q[1],0),
		  lv.max(q[2],0))+ lv.min(lv.max(lv.max(q[0],q[1]),q[2]),0)
# https://www.youtube.com/watch?v=-pdSjBPH3zM    

def lv_vecsub(a,b):
    return a[0]-b[0],a[1]-b[1],a[2]-b[2]

def lv_segment(c, a, b):
    v1=lv_vecsub(c,a)
    v2=lv_vecsub(b,a)
    n=lv_clamp(lv_dot(v1,v2)/lv_length(v2),0,1)
    d=lv_vecsub(c,lv_veclerp(a,b,n))
    return  lv_length(d)

def lv_union(a, b):
    return lv.min(a, b)

def lv_union_chamfer(a, b,r): # Credit: Doug Moen
    e = lv.max(r - lv.abs(a - b), 0);
    return lv.min( a, b) - e*0.5

def lv_union_ring(a, b,r):
    return lv.min(lv.min(a, b),lv.sqrt(a*a+b*b)-r)

def lv_union_groove(a, b,r):
    x=lv_clamp(r-lv.sqrt(a*a+b*b),0,r)
    return lv.min(a+x, b+x)

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

def lv_intersection_ring(a, b,r):
    return -lv_union_ring(-a,-b,r)

def lv_intersection_groove(a, b,r):
    return -lv_union_groove(-a,-b,r)

def lv_intersection_smooth(a, b,r):
    return -lv_union_smooth(-a,-b,r)

def lv_intersection_stairs(a, b,r,n):
    return -lv_union_stairs(-a,-b,r,n)

def lv_difference(a, b):
    return -lv_union(-a,b) 

def lv_difference_chamfer(a, b,r):
    return -lv_union_chamfer(-a,b,r)

def lv_difference_ring(a, b,r):
    return -lv_union_ring(-a,b,r)

def lv_difference_groove(a, b,r):
    return -lv_union_groove(-a,b,r)

def lv_difference_smooth(a, b,r):
    return -lv_union_smooth(-a,b,r)

def lv_difference_stairs(a, b,r,n):
    return -lv_union_stairs(-a,b,r,n)

def lv_juntion_ring(a,b,r):
    return lv.sqrt(a*a+b*b)-r

         
