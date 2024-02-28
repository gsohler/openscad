#pragma once

#include "node.h"
#include "Value.h"
#include <linalg.h>
#include "PolySet.h"

class DebugNode : public LeafNode
{
public:
  DebugNode(const ModuleInstantiation *mi) : LeafNode(mi) {}
  std::string toString() const override
  {
    std::ostringstream stream;
    stream << "debugnode( faces = ";
    for(int i=0;i<faces.size();i++) {
      stream << faces[i] ;
      stream << " ";
    }
    stream << ")";
    return  stream.str();
  }
  std::string name() const override { return "debug"; }
  std::unique_ptr<const Geometry> createGeometry() const override;
  std::vector<int>  faces;
};

