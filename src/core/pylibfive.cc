// Author: Sohler Guenther
// Date: 2023-01-01
// Purpose: Extend openscad with an python interpreter

#include <Python.h>
#include "pylibfive.h"
#include <libfive/tree/opcode.hpp>

using namespace libfive;

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


PyObject *python_lv_un_int(PyObject *self, PyObject *args, PyObject *kwargs,int op)
{
  char *kwlist[] = {"arg",  NULL};
  PyObject *arg = NULL;

  if (!PyArg_ParseTupleAndKeywords(args, kwargs, "O!", kwlist,
                                   &PyLibFiveType, &arg)) return NULL;

  libfive_tree tv = PyLibFiveObjectToTree(arg);
  libfive_tree res = libfive_tree_unary(op, tv);
  libfive_tree_stubs.push_back(res);
  return PyLibFiveObjectFromTree(&PyLibFiveType, res);
}

PyObject *python_lv_bin_int(PyObject *self, PyObject *args, PyObject *kwargs,int op)
{
#if 1
  char *kwlist[] = {"arg1","arg2",  NULL};
  PyObject *arg1 = NULL;
  PyObject *arg2 = NULL;

  if (!PyArg_ParseTupleAndKeywords(args, kwargs, "OO", kwlist,
                                   &arg1,
                                   &arg2
				   )) return NULL;

  libfive_tree a1 = PyLibFiveObjectToTree(arg1);
  libfive_tree a2 = PyLibFiveObjectToTree(arg2);
  libfive_tree res = libfive_tree_binary(op, a1,a2);
  libfive_tree_stubs.push_back(res);
#else
  int i;
  PyObject *obj = NULL;
  if(args == NULL) return Py_None;
  if(PyTuple_Size(args) == 0) return Py_None;
  obj= PyTuple_GetItem(args, 0);
//  Py_INCREF(obj);
  libfive_tree res = PyLibFiveObjectToTree(obj);
  for(i=1;i<PyTuple_Size(args);i++)
  {
  	obj= PyTuple_GetItem(args, i);
  	//Py_INCREF(obj);
  	libfive_tree tmp = PyLibFiveObjectToTree(obj);
  	libfive_tree res = libfive_tree_binary(op, res,tmp);
	libfive_tree_stubs.push_back(res);
  }
#endif
  return PyLibFiveObjectFromTree(&PyLibFiveType, res);
}

PyObject *python_lv_unop_int(PyObject *arg, int op)
{
  libfive_tree t = PyLibFiveObjectToTree(arg);

  libfive_tree res = libfive_tree_unary(op, t);
  libfive_tree_stubs.push_back(res);
  return PyLibFiveObjectFromTree(&PyLibFiveType, res);
}

PyObject *python_lv_binop_int(PyObject *arg1, PyObject *arg2, int op)
{
  libfive_tree t1 = PyLibFiveObjectToTree(arg1);
  libfive_tree t2 = PyLibFiveObjectToTree(arg2);

  libfive_tree res = libfive_tree_binary(op, t1, t2);
  libfive_tree_stubs.push_back(res);
  return PyLibFiveObjectFromTree(&PyLibFiveType, res);
}

PyObject *python_lv_x(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_void_int(self, args, kwargs,libfive_tree_x()); }
PyObject *python_lv_y(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_void_int(self, args, kwargs,libfive_tree_y()); }
PyObject *python_lv_z(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_void_int(self, args, kwargs,libfive_tree_z()); }
PyObject *python_lv_sqrt(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,Opcode::OP_SQRT); }
PyObject *python_lv_abs(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,Opcode::OP_ABS); }
PyObject *python_lv_max(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_bin_int(self, args, kwargs,Opcode::OP_MAX); }
PyObject *python_lv_min(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_bin_int(self, args, kwargs,Opcode::OP_MIN); }

PyObject *python_lv_sin(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,Opcode::OP_SIN); }
PyObject *python_lv_cos(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,Opcode::OP_COS); }
PyObject *python_lv_tan(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,Opcode::OP_TAN); }
PyObject *python_lv_asin(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,Opcode::OP_ASIN); }
PyObject *python_lv_acos(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,Opcode::OP_ACOS); }
PyObject *python_lv_atan(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,Opcode::OP_ATAN); }
PyObject *python_lv_exp(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,Opcode::OP_EXP); }
PyObject *python_lv_log(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,Opcode::OP_LOG); }

static PyMethodDef PyLibFiveFunctions[] = {
  {"x", (PyCFunction) python_lv_x, METH_VARARGS | METH_KEYWORDS, "Get X."},
  {"y", (PyCFunction) python_lv_y, METH_VARARGS | METH_KEYWORDS, "Get Y."},
  {"z", (PyCFunction) python_lv_z, METH_VARARGS | METH_KEYWORDS, "Get Z."},
  {"sqrt", (PyCFunction) python_lv_sqrt, METH_VARARGS | METH_KEYWORDS, "Square Root"},
  {"abs", (PyCFunction) python_lv_abs, METH_VARARGS | METH_KEYWORDS, "Absolute"},
  {"max", (PyCFunction) python_lv_max, METH_VARARGS | METH_KEYWORDS, "Maximal"},
  {"min", (PyCFunction) python_lv_min, METH_VARARGS | METH_KEYWORDS, "Minimal"},

  {"sin", (PyCFunction) python_lv_sin, METH_VARARGS | METH_KEYWORDS, "Sin"},
  {"cos", (PyCFunction) python_lv_cos, METH_VARARGS | METH_KEYWORDS, "Cos"},
  {"tan", (PyCFunction) python_lv_tan, METH_VARARGS | METH_KEYWORDS, "Tan"},
  {"asin", (PyCFunction) python_lv_asin, METH_VARARGS | METH_KEYWORDS, "Asin"},
  {"acos", (PyCFunction) python_lv_acos, METH_VARARGS | METH_KEYWORDS, "Acos"},
  {"atan", (PyCFunction) python_lv_atan, METH_VARARGS | METH_KEYWORDS, "Atan"},
  {"exp", (PyCFunction) python_lv_atan, METH_VARARGS | METH_KEYWORDS, "Exp"},
  {"log", (PyCFunction) python_lv_atan, METH_VARARGS | METH_KEYWORDS, "Log"},
  {NULL, NULL, 0, NULL}
};

PyObject *python_lv_add(PyObject *arg1, PyObject *arg2) { return python_lv_binop_int(arg1, arg2,  Opcode::OP_ADD); }
PyObject *python_lv_substract(PyObject *arg1, PyObject *arg2) { return python_lv_binop_int(arg1, arg2,  Opcode::OP_SUB); }
PyObject *python_lv_multiply(PyObject *arg1, PyObject *arg2) { return python_lv_binop_int(arg1, arg2,  Opcode::OP_MUL); }
PyObject *python_lv_remainder(PyObject *arg1, PyObject *arg2) { return python_lv_binop_int(arg1, arg2,  Opcode::OP_MOD); }
PyObject *python_lv_divide(PyObject *arg1, PyObject *arg2) { return python_lv_binop_int(arg1, arg2,  Opcode::OP_DIV); }
PyObject *python_lv_negate(PyObject *arg) { return python_lv_unop_int(arg, Opcode::OP_NEG); }


PyNumberMethods PyLibFiveNumbers =
{
  &python_lv_add,
  &python_lv_substract,
  &python_lv_multiply,
  &python_lv_remainder,
  0, /* binaryfunc nb_divmod; */
  0, /* ternaryfunc nb_power; */
  &python_lv_negate,
  0, /* unaryfunc nb_positive; */
  0, /* unaryfunc nb_absolute; */
  0, /* inquiry nb_bool; */
  0, /* unaryfunc nb_invert; */
  0, /* binaryfunc nb_lshift; */
  0, /* binaryfunc nb_rshift; */
  0, /* binaryfunc nb_and; */
  0, /* binaryfunc nb_xor; */
  0, /* binaryfunc nb_or; */
  0, /* unaryfunc nb_int; */
  0, /* void *nb_reserved; */
  0, /* unaryfunc nb_float; */

  0, /* binaryfunc nb_inplace_add; */
  0, /* binaryfunc nb_inplace_subtract; */
  0, /* binaryfunc nb_inplace_multiply; */
  0, /* binaryfunc nb_inplace_remainder; */
  0, /* ternaryfunc nb_inplace_power; */
  0, /* binaryfunc nb_inplace_lshift; */
  0, /* binaryfunc nb_inplace_rshift; */
  0, /* binaryfunc nb_inplace_and; */
  0, /* binaryfunc nb_inplace_xor; */
  0, /* binaryfunc nb_inplace_or; */

  0, /* binaryfunc nb_floor_divide; */
  &python_lv_divide, /* binaryfunc nb_true_divide; */
  0, /* binaryfunc nb_inplace_floor_divide; */
  0, /* binaryfunc nb_inplace_true_divide; */

  0, /* unaryfunc nb_index; */

  0, /* binaryfunc nb_matrix_multiply; */
  0, /* binaryfunc nb_inplace_matrix_multiply; */
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

