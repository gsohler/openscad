

class Curve
{
  public:
  int start, end;  
};

class ArcCurve :  public Curve
{
  public:
  ArcCurve() {
  }	  
  double r;
  Vector3d center, normdir;
};
