// Author: Sohler Guenther
// Date: 2023-01-01
// Purpose: Extend openscad with an python interpreter

#include <Python.h>
#include "pylibfive.h"

std::vector<libfive_tree> libfive_tree_stubs;
// https://docs.python.it/html/ext/dnt-basics.html

void PyLibFiveObject_dealloc(PyLibFiveObject *self)
{
//  Py_XDECREF(self->dict);
//  Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyObject *PyLibFiveObject_new(PyTypeObject *type, PyObject *args,  PyObject *kwds)
{
  PyLibFiveObject *self;
  self = (PyLibFiveObject *)  type->tp_alloc(type, 0);
  self->tree = 0;
  Py_XINCREF(self);
  return (PyObject *)self;
}

PyObject *PyLibFiveObjectFromTree(PyTypeObject *type, libfive_tree tree)
{
  PyLibFiveObject *self;
  self = (PyLibFiveObject *)  type->tp_alloc(type, 0);
  if (self != NULL) {
    self->tree = tree;
    Py_XINCREF(self);
    return (PyObject *)self;
  }
  return NULL;
}

libfive_tree PyLibFiveObjectToTree(PyObject *obj)
{
  libfive_tree result = NULL;
  if(obj != NULL && obj->ob_type == &PyLibFiveType) {
        result	= ((PyLibFiveObject *) obj)->tree;
  } else if(PyLong_Check(obj)) { 
	result=  libfive_tree_const(PyLong_AsLong(obj));
  	libfive_tree_stubs.push_back(result);
  } else if(PyFloat_Check(obj)) { 
	result=  libfive_tree_const(PyFloat_AsDouble(obj));
  	libfive_tree_stubs.push_back(result);
  } else {
	  printf("Unknown type! %p %p\n",obj->ob_type, &PyFloat_Type);
  }
//  Py_XDECREF(obj); TODO cannot activate
  return result;
}

static int PyLibFiveInit(PyLibFiveObject *self, PyObject *arfs, PyObject *kwds)
{
  return 0;
}

PyObject *python_lv_void_int(PyObject *self, PyObject *args, PyObject *kwargs,libfive_tree t)
{
  libfive_tree_stubs.push_back(t);
  return PyLibFiveObjectFromTree(&PyLibFiveType, t);
}


PyObject *python_lv_un_int(PyObject *self, PyObject *args, PyObject *kwargs,const char *op)
{
  char *kwlist[] = {"arg",  NULL};
  PyObject *arg = NULL;

  if (!PyArg_ParseTupleAndKeywords(args, kwargs, "O!", kwlist,
                                   &PyLibFiveType, &arg)) return NULL;

  libfive_tree tv = PyLibFiveObjectToTree(arg);
  libfive_tree res = libfive_tree_unary(libfive_opcode_enum(op), tv);
  libfive_tree_stubs.push_back(res);
  return PyLibFiveObjectFromTree(&PyLibFiveType, res);
}

PyObject *python_lv_bin_int(PyObject *self, PyObject *args, PyObject *kwargs,const char *op)
{
  char *kwlist[] = {"arg1","arg2",  NULL};
  PyObject *arg1 = NULL;
  PyObject *arg2 = NULL;

  if (!PyArg_ParseTupleAndKeywords(args, kwargs, "O!O!", kwlist,
                                   &PyLibFiveType, &arg1,
                                   &PyLibFiveType, &arg2
				   )) return NULL;

  libfive_tree a1 = PyLibFiveObjectToTree(arg1);
  libfive_tree a2 = PyLibFiveObjectToTree(arg2);
  libfive_tree res = libfive_tree_binary(libfive_opcode_enum(op), a1,a2);
  libfive_tree_stubs.push_back(res);
  return PyLibFiveObjectFromTree(&PyLibFiveType, res);
}

PyObject *python_lv_op_int(PyObject *arg1, PyObject *arg2, const char *op)
{
  libfive_tree t1 = PyLibFiveObjectToTree(arg1);
  libfive_tree t2 = PyLibFiveObjectToTree(arg2);

  libfive_tree res = libfive_tree_binary(libfive_opcode_enum(op), t1, t2);
  libfive_tree_stubs.push_back(res);
  return PyLibFiveObjectFromTree(&PyLibFiveType, res);
}

PyObject *python_lv_x(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_void_int(self, args, kwargs,libfive_tree_x()); }
PyObject *python_lv_y(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_void_int(self, args, kwargs,libfive_tree_y()); }
PyObject *python_lv_z(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_void_int(self, args, kwargs,libfive_tree_z()); }
PyObject *python_lv_add(PyObject *arg1, PyObject *arg2) { return python_lv_op_int(arg1, arg2,  "add"); }
PyObject *python_lv_substract(PyObject *arg1, PyObject *arg2) { return python_lv_op_int(arg1, arg2,  "sub"); }
PyObject *python_lv_multiply(PyObject *arg1, PyObject *arg2) { return python_lv_op_int(arg1, arg2,  "mul"); }
PyObject *python_lv_divide(PyObject *arg1, PyObject *arg2) { return python_lv_op_int(arg1, arg2,  "div"); }
PyObject *python_lv_sqrt(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,"sqrt"); }
PyObject *python_lv_abs(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,"abs"); }
PyObject *python_lv_max(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_bin_int(self, args, kwargs,"max"); }
PyObject *python_lv_min(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_bin_int(self, args, kwargs,"min"); }

static PyMethodDef PyLibFiveFunctions[] = {
  {"libfive_x", (PyCFunction) python_lv_x, METH_VARARGS | METH_KEYWORDS, "Get X."},
  {"libfive_y", (PyCFunction) python_lv_y, METH_VARARGS | METH_KEYWORDS, "Get Y."},
  {"libfive_z", (PyCFunction) python_lv_z, METH_VARARGS | METH_KEYWORDS, "Get Z."},
  {"libfive_sqrt", (PyCFunction) python_lv_sqrt, METH_VARARGS | METH_KEYWORDS, "Square Root"},
  {"libfive_abs", (PyCFunction) python_lv_sqrt, METH_VARARGS | METH_KEYWORDS, "Absolute"},
  {"libfive_max", (PyCFunction) python_lv_max, METH_VARARGS | METH_KEYWORDS, "Maximal"},
  {"libfive_min", (PyCFunction) python_lv_min, METH_VARARGS | METH_KEYWORDS, "Minimal"},
  {NULL, NULL, 0, NULL}
};



PyNumberMethods PyLibFiveNumbers =
{
  &python_lv_add,
  &python_lv_substract,
  &python_lv_multiply,
  &python_lv_divide
};

PyTypeObject PyLibFiveType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "PyLibFive",             			/* tp_name */
    sizeof(PyLibFiveObject), 			/* tp_basicsize */
    0,                         			/* tp_itemsize */
    (destructor) PyLibFiveObject_dealloc,	/* tp_dealloc */
    0,                         			/* tp_print */
    0,                         			/* tp_getattr */
    0,                         			/* tp_setattr */
    0,                         			/* tp_reserved */
    0,                         			/* tp_repr */
    &PyLibFiveNumbers, 				/* tp_as_number */
    0,                         			/* tp_as_sequence */
    0,		        			/* tp_as_mapping */
    0,                         			/* tp_hash  */
    0,                         			/* tp_call */
    0,                         			/* tp_str */
    0,                         			/* tp_getattro */
    0,                         			/* tp_setattro */
    0,                         			/* tp_as_buffer */
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,	/* tp_flags */
    "PyLibFive Object",          		/* tp_doc */
    0,                         			/* tp_traverse */
    0,                         			/* tp_clear */
    0,                         			/* tp_richcompare */
    0,                         			/* tp_weaklistoffset */
    0,                         			/* tp_iter */
    0,                         			/* tp_iternext */
    0,			             		/* tp_methods */
    0,             				/* tp_members */
    0,                         			/* tp_getset */
    0,                         			/* tp_base */
    0,                         			/* tp_dict */
    0,                         			/* tp_descr_get */
    0,                         			/* tp_descr_set */
    0,                         			/* tp_dictoffset */
    (initproc) PyLibFiveInit,      		/* tp_init */
    0,                         			/* tp_alloc */
    PyLibFiveObject_new,                	/* tp_new */
};


static PyModuleDef LibFiveModule = {
  PyModuleDef_HEAD_INIT,
  "LibFive",
  "Example module that creates an extension type.",
  -1,
  PyLibFiveFunctions,
  NULL, NULL, NULL, NULL
};

PyMODINIT_FUNC PyInit_PyLibFive(void)
{
  PyObject *m;
  if (PyType_Ready(&PyLibFiveType) < 0) return NULL;
  m = PyModule_Create(&LibFiveModule);
  if (m == NULL) return NULL;

  Py_INCREF(&PyLibFiveType);
  PyModule_AddObject(m, "openscad", (PyObject *)&PyLibFiveType);
  return m;
}

PyObject *PyInit_libfive(void)
{
  return PyModule_Create(&LibFiveModule);
}

