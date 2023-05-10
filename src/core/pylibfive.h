#include <Python.h>
#include <memory>
#include <libfive.h>

#pragma GCC diagnostic ignored "-Wwrite-strings"

extern std::vector<libfive_tree> libfive_tree_stubs;

typedef struct {
  PyObject_HEAD
  libfive_tree tree;  
  /* Type-specific fields go here. */
} PyLibFiveObject;

PyMODINIT_FUNC PyInit_PyLibFive(void);

extern PyTypeObject PyLibFiveType;

void PyLibFiveObject_dealloc(PyLibFiveObject *self);

PyObject *PyLibFiveObjectFromTree(PyTypeObject *type, const std::vector<libfive_tree> &tree);
std::vector<libfive_tree> PyLibFiveObjectToTree(PyObject *object);

