#pragma once
#include "node.h"
#include "src/core/function.h"
#include "src/geometry/Polygon2d.h"

extern bool python_active;
extern bool python_trusted;

void initPython(void);

void finishPython();

std::string evaluatePython(const std::string &code,
			   double time,
                           AssignmentList &assignments);

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
extern std::shared_ptr<AbstractNode> python_result_node;
