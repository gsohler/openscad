#include <Python.h>
#include <memory>
#include <libfive.h>

#pragma GCC diagnostic ignored "-Wwrite-strings"

#define DECLARE_INSTANCE	std::string instance_name; \
	AssignmentList inst_asslist;\
	ModuleInstantiation *instance = new ModuleInstantiation(instance_name,inst_asslist, Location::NONE);


typedef struct {
  PyObject_HEAD
  libfive_tree tree;  
  /* Type-specific fields go here. */
} PyLibFiveObject;

PyMODINIT_FUNC PyInit_PyLibFive(void);

extern PyTypeObject PyLibFiveType;

void PyLibFiveObject_dealloc(PyLibFiveObject *self);

PyObject *PyLibFiveObjectFromTree(PyTypeObject *type, libfive_tree tree);
libfive_tree PyLibFiveObjectToTree(PyObject *object);

