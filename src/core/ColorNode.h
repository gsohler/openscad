#pragma once

#include <string>

#include "core/node.h"
#include "geometry/linalg.h"

class ColorNode : public AbstractNode
{
public:
  VISITABLE();
  ColorNode(const ModuleInstantiation *mi) : AbstractNode(mi), color(-1.0f, -1.0f, -1.0f, 1.0f), textureind(0) { }
  std::string toString() const override;
  std::string name() const override;

  Color4f color;
  int textureind;
};
