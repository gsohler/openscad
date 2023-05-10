import libfive as lv

#http://www.gradientspace.com/tutorials/category/g3sharp

def lv_coord():
        return lv.x(),lv.y(),lv.z()

def lv_clamp(c,low, high):
    return lv.min(lv.max(c,low),high)

def lv_lerp(a,b,t):
    return  a*(1-t) + b*t

def lv_trans(c,v):
    return c[0]-v[0] ,c[1]-v[1],c[2]-v[2]

def lv_matrix_sub(c,p, n):
    return (lv_clamp(c+p/2,0,p*n)%p)-p/2

def lv_matrix(c,p, n):
    return lv_matrix_sub(c[0],p[0],n[0]), lv_matrix_sub(c[1],p[1],n[1]), lv_matrix_sub(c[2],p[2],n[2])

def lv_scalar(a,b):
    return a[0]*b[0]+a[1]*b[1]+a[2]*b[2]

def lv_len(n):
    return lv.sqrt(n[0]*n[0]+n[1]*n[1]+n[2]*n[2])

def lv_vec_scale(v,n):
    return v[0]*n, v[1]*n, v[2]*n

def lv_vec_unit(v):
    return lv_vec_scale(v,1.0/lv_len(v))

def lv_mirror(c,n1):
    n=lv_vec_unit(n1)
    e =lv_scalar(c,n)
    x=n*(lv.abs(e)-e)
    return c[0]+x[0],c[1]+x[1],c[2]+x[2]

def lv_cubemirror(c):
    xa=lv.abs(c[0])
    ya=lv.abs(c[1])
    za=lv.abs(c[2])
    xd=lv.max(lv.comp(xa,lv.max(ya,za)),0)
    yd=lv.max(lv.comp(ya,lv.max(xa,za)),0)
    zd= lv.max(lv.comp(za,lv.max(xa,ya)),0)
    return xd*c[1]+yd*c[2]+zd*c[0],xd*c[2]+yd*c[0]+zd*c[1],xd*xa+yd*ya+zd*za

def lv_sphere(c,r):
    return lv.sqrt(c[0]*c[0]+c[1]*c[1]+c[2]*c[2])-r

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

         
