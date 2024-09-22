#pragma once

#include <string>

#include "node.h"
#include "enums.h"

class CsgOpNode : public AbstractNode
{
public:
  VISITABLE();
  OpenSCADOperator type;
  double r=0.0; // Radius for fillets
  int fn=2;
  CsgOpNode(const ModuleInstantiation *mi, OpenSCADOperator type) : AbstractNode(mi), type(type) { }
  std::string toString() const override;
  std::string name() const override;
};
