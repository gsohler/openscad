#pragma once
#include "node.h"
#include "src/core/function.h"
#include "src/geometry/Polygon2d.h"
#include <Selection.h>

void initJs(double time);
void finishJs();
std::string evaluateJs(const std::string &code);

extern std::shared_ptr<AbstractNode> js_result_node;
#if 0
std::shared_ptr<AbstractNode>
python_modulefunc(const ModuleInstantiation *module,
                  const std::shared_ptr<const Context> &context,
                  int *modulefound);

Value python_functionfunc(const FunctionCall *call,
                          const std::shared_ptr<const Context> &context);
double python_doublefunc(void *cbfunc, double arg);
Outline2d python_getprofile(void *cbfunc, int fn, double arg);

extern bool pythonMainModuleInitialized;
extern bool pythonRuntimeInitialized;
#endif
