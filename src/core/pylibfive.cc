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

PyObject *PyLibFiveObjectFromTree(PyTypeObject *type, const std::vector<libfive_tree> &tree)
{
  if(tree.size() == 0) {
    return Py_None;	  
  } else if(tree.size() == 1) {
    PyLibFiveObject *res;
    res = (PyLibFiveObject *)  type->tp_alloc(type, 0);
    if (res != NULL) {
      res->tree = tree[0];
      Py_XINCREF(res);
      return (PyObject *)res;
    }
  } else {
    PyObject  *res = PyTuple_New(tree.size());
    for(int i=0;i<tree.size();i++) {
      PyLibFiveObject *sub;
      sub = (PyLibFiveObject *)  type->tp_alloc(type, 0);
      if (sub != NULL) {
        sub->tree = tree[i];
        Py_XINCREF(sub);
	PyTuple_SetItem(res,i,(PyObject *) sub);
      }
    }    
    return res;
  }	  

  return Py_None;
}

std::vector<libfive_tree> PyLibFiveObjectToTree(PyObject *obj)
{
  std::vector<libfive_tree> result;
  if(obj != NULL && obj->ob_type == &PyLibFiveType) {
        result.push_back(((PyLibFiveObject *) obj)->tree);
  } else if(PyLong_Check(obj)) { 
	result.push_back(libfive_tree_const(PyLong_AsLong(obj)));
  	libfive_tree_stubs.push_back(result[0]);
  } else if(PyFloat_Check(obj)) { 
	result.push_back(libfive_tree_const(PyFloat_AsDouble(obj)));
  	libfive_tree_stubs.push_back(result[0]);
  } else if(PyTuple_Check(obj)){
	  for(int i=0;i<PyTuple_Size(obj);i++) {
		PyObject *x=PyTuple_GetItem(obj,i);
		std::vector<libfive_tree> sub = PyLibFiveObjectToTree(x);
		result.insert(result.end(), sub.begin(), sub.end());
	  }
  } else if(PyList_Check(obj)){
	  for(int i=0;i<PyList_Size(obj);i++) {
		PyObject *x=PyList_GetItem(obj,i);
		std::vector<libfive_tree> sub = PyLibFiveObjectToTree(x);
		result.insert(result.end(), sub.begin(), sub.end());
	  }
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
  std::vector<libfive_tree> vec;
  vec.push_back(t);
  return PyLibFiveObjectFromTree(&PyLibFiveType, vec);
}


PyObject *python_lv_un_int(PyObject *self, PyObject *args, PyObject *kwargs,int op)
{
  char *kwlist[] = {"arg",  NULL};
  PyObject *arg = NULL;

  if (!PyArg_ParseTupleAndKeywords(args, kwargs, "O", kwlist, &arg)) return NULL;

  std::vector<libfive_tree> tv = PyLibFiveObjectToTree(arg);
  std::vector<libfive_tree> res;
  for(int i=0;i<tv.size();i++) {
    libfive_tree sub = libfive_tree_unary(op, tv[i]);
    libfive_tree_stubs.push_back(sub);
    res.push_back(sub);
  }
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

  std::vector<libfive_tree> a1 = PyLibFiveObjectToTree(arg1);
  std::vector<libfive_tree> a2 = PyLibFiveObjectToTree(arg2);
  std::vector<libfive_tree> res;
  if(a1.size() == a2.size()) {
    for(int i=0;i<a1.size();i++) {		 
      res.push_back(libfive_tree_binary(op, a1[i],a2[i]));
    }
  }
  else if(a2.size() == 1) {
    for(int i=0;i<a1.size();i++) {		 
      res.push_back(libfive_tree_binary(op, a1[i],a2[0]));
    }
  } else { 
    printf("Cannot handle  bin %d binop %d \n",a1.size(), a2.size());
    return Py_None;		    
  }
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
  std::vector<libfive_tree> t = PyLibFiveObjectToTree(arg);

  std::vector<libfive_tree> res;
  for(int i=0;i<t.size();i++) {
    res.push_back(libfive_tree_unary(op, t[i]));
  }  
  return PyLibFiveObjectFromTree(&PyLibFiveType, res);
}

PyObject *python_lv_binop_int(PyObject *arg1, PyObject *arg2, int op)
{
  std::vector<libfive_tree> t1 = PyLibFiveObjectToTree(arg1);
  std::vector<libfive_tree> t2 = PyLibFiveObjectToTree(arg2);

  std::vector<libfive_tree> res ;
  if(t1.size() == t2.size()) {
    for(int i=0;i<t1.size();i++) {		 
      res.push_back(libfive_tree_binary(op, t1[i],t2[i]));
    }
  }
  else if(t2.size() == 1) {
    for(int i=0;i<t1.size();i++) {		 
      res.push_back(libfive_tree_binary(op, t1[i],t2[0]));
    }
  } else { 
    printf("Cannot handle  bin %d binop %d \n",t1.size(), t2.size());
    return Py_None;		    
  }
  libfive_tree_stubs.insert(libfive_tree_stubs.end(), res.begin(), res.end());

  return PyLibFiveObjectFromTree(&PyLibFiveType, res);
}

PyObject *python_lv_x(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_void_int(self, args, kwargs,libfive_tree_x()); }
PyObject *python_lv_y(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_void_int(self, args, kwargs,libfive_tree_y()); }
PyObject *python_lv_z(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_void_int(self, args, kwargs,libfive_tree_z()); }
PyObject *python_lv_sqrt(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,Opcode::OP_SQRT); }
PyObject *python_lv_abs(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,Opcode::OP_ABS); }
PyObject *python_lv_max(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_bin_int(self, args, kwargs,Opcode::OP_MAX); }
PyObject *python_lv_min(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_bin_int(self, args, kwargs,Opcode::OP_MIN); }
PyObject *python_lv_pow(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_bin_int(self, args, kwargs,Opcode::OP_POW); }
PyObject *python_lv_comp(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_bin_int(self, args, kwargs,Opcode::OP_COMPARE); }

PyObject *python_lv_sin(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,Opcode::OP_SIN); }
PyObject *python_lv_cos(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,Opcode::OP_COS); }
PyObject *python_lv_tan(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,Opcode::OP_TAN); }
PyObject *python_lv_asin(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,Opcode::OP_ASIN); }
PyObject *python_lv_acos(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,Opcode::OP_ACOS); }
PyObject *python_lv_atan(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,Opcode::OP_ATAN); }
PyObject *python_lv_exp(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,Opcode::OP_EXP); }
PyObject *python_lv_log(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,Opcode::OP_LOG); }

PyObject *python_lv_print(PyObject *self, PyObject *args, PyObject *kwargs)
{
  char *kwlist[] = {"arg",  NULL};
  PyObject *arg = NULL;

  if (!PyArg_ParseTupleAndKeywords(args, kwargs, "O!", kwlist, &PyLibFiveType, &arg)) return NULL;
  std::vector<libfive_tree> tv = PyLibFiveObjectToTree(arg);
  for(int i=0;i<tv.size();i++){
    printf("tree %d: %s\n",i,libfive_tree_print(tv[i]));
  }
  return Py_None;
}

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
  {"exp", (PyCFunction) python_lv_exp, METH_VARARGS | METH_KEYWORDS, "Exp"},
  {"log", (PyCFunction) python_lv_log, METH_VARARGS | METH_KEYWORDS, "Log"},
  {"pow", (PyCFunction) python_lv_pow, METH_VARARGS | METH_KEYWORDS, "Power"},
  {"comp", (PyCFunction) python_lv_comp, METH_VARARGS | METH_KEYWORDS, "Compare"},
  {"print", (PyCFunction) python_lv_print, METH_VARARGS | METH_KEYWORDS, "Print"},
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

