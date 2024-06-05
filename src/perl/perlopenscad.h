#include <memory>
#include "node.h"
#include <geometry/Polygon2d.h>
#include "src/core/function.h"

#pragma GCC diagnostic ignored "-Wwrite-strings"

#define DECLARE_INSTANCE	std::string instance_name; \
	AssignmentList inst_asslist;\
	ModuleInstantiation *instance = new ModuleInstantiation(instance_name,inst_asslist, Location::NONE);


void initPerl(double time);
std::string evaluatePerl(const std::string & code);
void finishPerl(void);


