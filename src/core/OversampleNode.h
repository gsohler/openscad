#pragma once

#include "node.h"
#include "Value.h"
#include <linalg.h>
#include "PolySet.h"

class OversampleNode : public LeafNode
{
public:
  OversampleNode(const ModuleInstantiation *mi) : LeafNode(mi) {}
  std::string toString() const override
  {
    std::ostringstream stream;
    stream << "oversample( " << n << ")";
    return  stream.str();
  }
  std::string name() const override { return "oversample"; }
  const Geometry *createGeometry() const override;
  int n;
};

