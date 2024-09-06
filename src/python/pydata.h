#include <Python.h>
#include <memory>
#include <libfive.h>

#pragma GCC diagnostic ignored "-Wwrite-strings"

typedef struct {
  PyObject_HEAD
  libfive::Tree *tree;
  /* Type-specific fields go here. */
} PyDataObject;

PyMODINIT_FUNC PyInit_PyData(void);

extern PyTypeObject PyDataType;

PyObject *PyDataObjectFromTree(PyTypeObject *type, const std::vector<libfive::Tree *> &tree);
std::vector<libfive::Tree *> PyDataObjectToTree(PyObject *object);

