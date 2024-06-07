#include <memory>
#include "node.h"
#include <geometry/Polygon2d.h>
#include "src/core/function.h"
#include "mujs.h"

#pragma GCC diagnostic ignored "-Wwrite-strings"

#define DECLARE_INSTANCE	std::string instance_name; \
	AssignmentList inst_asslist;\
	ModuleInstantiation *instance = new ModuleInstantiation(instance_name,inst_asslist, Location::NONE);

extern js_State *js_interp;
void registerJsFunctions(void);
void initJs(double time);
std::string evaluateJs(const std::string & code);
void finishJs(void);


