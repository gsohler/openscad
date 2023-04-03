#include <Python.h>
#include <memory>
#include <libfive.h>

#pragma GCC diagnostic ignored "-Wwrite-strings"

typedef struct {
  PyObject_HEAD
  libfive::Tree *tree;
  /* Type-specific fields go here. */
} PyLibFiveObject;

PyMODINIT_FUNC PyInit_PyLibFive(void);

extern PyTypeObject PyLibFiveType;

void PyLibFiveObject_dealloc(PyLibFiveObject *self);

PyObject *PyLibFiveObjectFromTree(PyTypeObject *type, libfive::Tree *tree);
libfive::Tree *PyLibFiveObjectToTree(PyObject *object);

