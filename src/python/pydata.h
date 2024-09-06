#include <Python.h>
#include <memory>
#ifdef ENABLE_LIBFIVE
#include <libfive.h>
#endif

#include <src/core/module.h>
#include <boost/optional.hpp>

#pragma GCC diagnostic ignored "-Wwrite-strings"

#define DATA_TYPE_UNKNOWN -1
#define DATA_TYPE_LIBFIVE 0
#define DATA_TYPE_SCADMODULE 0

typedef struct {
  PyObject_HEAD
#ifdef ENABLE_LIBFIVE	  
  void *data;
  int data_type;  
//  libfive::Tree *tree;
#endif  
  /* Type-specific fields go here. */
} PyDataObject;

PyMODINIT_FUNC PyInit_PyData(void);

extern PyTypeObject PyDataType;

#ifdef ENABLE_LIBFIVE
PyObject *PyDataObjectFromTree(PyTypeObject *type, const std::vector<libfive::Tree *> &tree);
std::vector<libfive::Tree *> PyDataObjectToTree(PyObject *object);
#endif

PyObject *PyDataObjectFromModule(PyTypeObject *type, boost::optional<InstantiableModule> mod);
boost::optional<InstantiableModule> PyDataObjectToModule(PyObject *obj);


