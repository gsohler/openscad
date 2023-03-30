import libfive as lv

def lv_coord():
    return lv.x(),lv.y(),lv.z()

def lv_trans(c,v):
    return c[0]-v[0] ,c[1]-v[1],c[2]-v[2]

def lv_sphere(c,r):
    return c[0]*c[0]+c[1]*c[1]+c[2]*c[2]-r*r

def lv_union(ob1, ob2):
    return lv.min(ob1, ob2)

def lv_union_chamfer(ob1, ob2,r):
    e = lv.max(r - lv.abs(ob1 - ob2), 0);
    return lv.min( ob1, ob2) - e*0.5

def lv_difference(ob1, ob2):
    return lv.max(ob1 ,-ob2)

def lv_intersection(ob1, ob2):
    return lv.max(ob1, ob2)

def lv_box(c,dim):
    return lv.min(lv.min(c[0]/dim[0],c[1]/dim[1]),c[2]/dim[2])
         
