#pragma once

#include "node.h"
#include "Value.h"
#include <linalg.h>
#include "PolySet.h"

#ifdef ENABLE_PYTHON
#include <Python.h>
#endif
#include <libfive.h>

class FrepNode : public LeafNode
{
public:
  FrepNode(const ModuleInstantiation *mi) : LeafNode(mi) {}
  std::string toString() const override
  {
    std::ostringstream stream;
    stream << "sdf( " << rand() << ")";
    return  stream.str();
  }
  std::string name() const override { return "sdf"; }
  const Geometry *createGeometry() const override;
 #ifdef ENABLE_PYTHON
  PyObject *expression;
 #endif  
  double x1,y1,z1;
  double x2,y2,z2;
  double res;
};

