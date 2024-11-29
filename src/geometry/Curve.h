#pragma once

class Curve
{
  public:
  int start, end;  
  virtual void display(std::vector<Vector3d> &vertices);
  virtual void reverse(void);
  virtual int operator==(const Curve &other);
};

class ArcCurve :  public Curve
{
  public:
  ArcCurve(Vector3d center, Vector3d normdir, double r);
  void display(std::vector<Vector3d> &vertices);
  void display(void);
  void reverse(void);
  int operator==(const ArcCurve &other);

  double r;
  Vector3d center, normdir;
};
