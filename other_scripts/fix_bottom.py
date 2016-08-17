import numpy as np

class Edges(dict): 
  def __init__(self): 
    pass

  def add(self,a,b,f): 
    key = (a,b) if a < b else (b,a)
    if not key in self:
      self[key] = [None,None]
    if key == (a,b):
      self[key][0] = f
    else: 
      self[key][1] = f

class Model: 
  def __init__(self): 
    self.vs = [] # Vertex List
    self.fs = [] # Face list
    self.ns = [] # Normal list
    self.edges = []

  def compute_edges_and_normals(self):
    edges = Edges()
    faces = [set() for x in self.fs]
    Cs = []
    for fi,f in enumerate(self.fs):
      ai,bi,ci = f
      va,vb,vc = self.vs[list(f)]
      v1 = va-vb
      v2 = va-vc
      cs = (va+vb+vc)/3.0
      Cs.append(cs)
      n = np.cross(v1,v2)
      edges.add(ai,bi,fi)
      edges.add(bi,ci,fi)
      edges.add(ci,ai,fi)
      faces[ai].add(fi)
      faces[bi].add(fi)
      faces[ci].add(fi)
      self.ns.append(n)
    self.faces = faces
    self.Cs = np.array(Cs)
    self.Us = (self.Cs.T/np.linalg.norm(self.Cs,axis=1)).T
    self.ns = np.array(self.ns)
    self.ns /= np.linalg.norm(self.ns,axis=1)[...,np.newaxis]
    for (vai,vbi),(fai,fbi) in edges.items(): 
      if fai == None or fbi == None: continue
      self.edges.append((vai,vbi,fai,fbi))
    self.edges = np.array(self.edges)

  def save_obj(self, filename,edit_vs = False):
    non_vs_lines = []
    if edit_vs: 
      non_vs_lines = self.non_vs_lines
    save_obj(self.vs,self.fs,filename,non_vs_lines)
 
  def add_vertex(self, x, y, z): 
    self.vs.append((x, y, z))
    return len(self.vs) - 1

  def add_triangle(self, i1, i2, i3): 
    self.fs.append((i1, i2, i3))

  def add_quad(self, i1, i2, i3, i4): 
    self.add_triangle(i1, i2, i3)
    self.add_triangle(i3, i4, i1)

  def translate(self, x, y, z): 
    self.vs = self.vs + np.array((x, y, z))
  
  def rotate_x(self, theta): 
    Rx = pc.Rx(np.deg2rad(theta))
    self.vs = np.dot(Rx, self.vs.T).T
    
  def rotate_y(self, theta): 
    Ry = pc.Ry(np.deg2rad(theta))
    self.vs = np.dot(Ry, self.vs.T).T

  def rotate_z(self, theta): 
    Rz = pc.Rz(np.deg2rad(theta))
    self.vs = np.dot(Rz, self.vs.T).T
  
  def scale(self, s): 
    self.vs = self.vs * s

  def copy(self): 
    obj = Model()
    obj.vs = self.vs.copy()
    return obj
 
  def read_obj(self, filename, edit_vs=False):
    fh = open(filename, 'r')
    nv = 0;
    nf = 0;
    cv = 0;
    if edit_vs: 
      self.non_vs_lines = []
    for line in fh.readlines():
      els = line.split();
      if els[0] != 'v' and edit_vs: 
        self.non_vs_lines.append(line.strip())
      if len(els) != 4 and len(els) != 5 and len(els) != 7: continue
      if els[0] == 'v': nv += 1
      if els[0] == 'f': nf += 1
    fh.close();
    fh = open(filename, "r")
    self.nv = nv
    self.nf = nf
    self.vs = np.zeros((nv, 3), 'd')
    self.colors = np.ones((nv, 3), "d") * 255
    for line in fh.readlines():
      if line[0] == "#": continue
      els = line.split();
      if len(els) != 4 and len(els) != 5 and len(els) != 7: continue
      vs = els[1:]
      if els[0] == 'v':
        vs = np.array([float(v) for v in vs])
        self.vs[cv] = vs[:3]
      if len(vs) == 6:
        self.colors[cv] = (vs[3:] * 255).astype(int)
      cv += 1
      if els[0] == 'f':
        f = []
        for v in vs: 
          vi = v.split("/")[0]
          f.append(int(vi) - 1)
        if len(f) == 4:
          self.add_quad(*f)
        if len(f) == 3:
          self.add_triangle(*f)
  
  def finalize(self):
    self.fs = np.array(self.fs)
    self.vs = np.array(self.vs)
    self.compute_edges_and_normals()

  def get_face_xs(self): 
    return np.array([self.vs[list(f)].T[0] for f in self.fs])
  
  def get_face_ys(self): 
    return np.array([self.vs[list(f)].T[1] for f in self.fs])


  def get_face_zs(self): 
    return np.array([self.vs[list(f)].T[2] for f in self.fs])

def ccw(Axy,Bxy,Cxy): 
  x1,y1 = Axy
  x2,y2 = Bxy
  x3,y3 = Cxy
  return np.sign(-x2*y1+x3*y1+x1*y2-x3*y2-x1*y3+x2*y3)


def filter_below_z(mesh,z_min):
  below_vertices = mesh.vs.T[-1] <= z_min
  mesh.vs[below_vertices].T[-1] = z_min
  x,y,_ = mesh.vs[below_vertices].mean(0)
  mesh.vs[below_vertices] = (x,y,z_min)

def load_obj(filename,edit_vs=False): 
  mesh = Model()
  mesh.read_obj(filename,edit_vs=edit_vs)
  return mesh
  
def save_obj(vs,fs,filename,non_vs_lines=[]):
  if len(non_vs_lines) > 0: 
    with open(filename, "w") as fh: 
      print >> fh, non_vs_lines[0] #Assume first line is mtllib 
      for v in vs: 
        x, y, z = tuple(v)
        print >> fh, "v %f %f %f" % (x, y, z)
      for l in non_vs_lines[1:]: 
        print >> fh, l
  else: 
    with open(filename, "w") as fh: 
      for v in vs: 
        x, y, z = tuple(v)
        print >> fh, "v %f %f %f" % (x, y, z)
      for f in fs:
          print >> fh, "f %d %d %d" % tuple(np.array(f) + 1)

if __name__ == "__main__": 
  import sys
  i_fn,o_fn,z_min = sys.argv[1:4]
  mesh = load_obj(i_fn,edit_vs=True)
  filter_below_z(mesh,float(z_min))
  mesh.save_obj(o_fn,edit_vs=True)

