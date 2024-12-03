// Author: Sohler Guenther
// Date: 2023-01-01
// Purpose: Extend openscad with an python interpreter

#include <Python.h>
#include "pydata.h"

#include "Assignment.h"
#include "ModuleInstantiation.h"
#include "BuiltinContext.h"
#include "Expression.h"
PyObject *PyOpenSCADObjectFromNode(PyTypeObject *type, const std::shared_ptr<AbstractNode> &node);

#ifdef ENABLE_LIBFIVE
#include <libfive/tree/opcode.hpp>
#endif

// https://docs.python.it/html/ext/dnt-basics.html

void PyDataObject_dealloc(PyObject *self)
{
//  Py_XDECREF(self->dict);
//  Py_TYPE(self)->tp_free((PyObject *)self);
}

static PyObject *PyDataObject_new(PyTypeObject *type, PyObject *args,  PyObject *kwds)
{
  PyDataObject *self;
  self = (PyDataObject *)  type->tp_alloc(type, 0);
  self->data_type=DATA_TYPE_UNKNOWN;
  self->data = 0;
  Py_XINCREF(self);
  return (PyObject *)self;
}

PyObject *PyDataObjectFromModule(PyTypeObject *type, boost::optional<InstantiableModule> mod)
{
  PyDataObject *res;
  res = (PyDataObject *)  type->tp_alloc(type, 0);
  if (res != NULL) {
    res->data_type = DATA_TYPE_SCADMODULE;
    res->data = (void *) mod->module;
    Py_XINCREF(res);
    return (PyObject *)res;
  }
  return Py_None;
}

boost::optional<InstantiableModule> PyDataObjectToModule(PyObject *obj)
{
  if(obj != NULL && obj->ob_type == &PyDataType) {
    PyDataObject * dataobj = (PyDataObject *) obj;
    if(dataobj->data_type == DATA_TYPE_SCADMODULE) {
     InstantiableModule m;
     m.module=(AbstractModule *)  dataobj->data;
     boost::optional<InstantiableModule> res(m);
     return res;
    }
  }
  boost::optional<InstantiableModule> res;
  return res;
}


#ifdef ENABLE_LIBFIVE

PyObject *PyDataObjectFromTree(PyTypeObject *type, const std::vector<libfive::Tree *> &tree)
{
  if(tree.size() == 0) {
    return Py_None;	  
  } else if(tree.size() == 1) {
    PyDataObject *res;
    res = (PyDataObject *)  type->tp_alloc(type, 0);
    if (res != NULL) {
      res->data = (void *) tree[0];
      res->data_type = DATA_TYPE_LIBFIVE;
      Py_XINCREF(res);
      return (PyObject *)res;
    }
  } else {
    PyObject  *res = PyTuple_New(tree.size());
    for(int i=0;i<tree.size();i++) {
      PyDataObject *sub;
      sub = (PyDataObject *)  type->tp_alloc(type, 0);
      if (sub != NULL) {
        sub->data = tree[i];
	sub->data_type = DATA_TYPE_LIBFIVE;
        Py_XINCREF(sub);
	PyTuple_SetItem(res,i,(PyObject *) sub);
      }
    }    
    return res;
  }	  

  return Py_None;
}

std::vector<libfive::Tree *> PyDataObjectToTree(PyObject *obj)
{
  std::vector<libfive::Tree *> result;
  if(obj != NULL && obj->ob_type == &PyDataType) {
	PyDataObject * dataobj = (PyDataObject *) obj;
        if(dataobj->data_type == DATA_TYPE_LIBFIVE) result.push_back((libfive::Tree *) dataobj->data);
  } else if(PyLong_Check(obj)) { 
  	result.push_back(new libfive::Tree(libfive::Tree(PyLong_AsLong(obj))));
  } else if(PyFloat_Check(obj)) { 
  	result.push_back(new libfive::Tree(libfive::Tree(PyFloat_AsDouble(obj))));
  } else if(PyTuple_Check(obj)){
	  for(int i=0;i<PyTuple_Size(obj);i++) {
		PyObject *x=PyTuple_GetItem(obj,i);
		std::vector<libfive::Tree *> sub = PyDataObjectToTree(x);
		result.insert(result.end(), sub.begin(), sub.end());
	  }
  } else if(PyList_Check(obj)){
	  for(int i=0;i<PyList_Size(obj);i++) {
		PyObject *x=PyList_GetItem(obj,i);
		std::vector<libfive::Tree *> sub = PyDataObjectToTree(x);
		result.insert(result.end(), sub.begin(), sub.end());
	  }
  } else {
	  printf("Unknown type! %p %p\n",obj->ob_type, &PyFloat_Type);
  }
//  Py_XDECREF(obj); TODO cannot activate
  return result;
}

#endif
static int PyDataInit(PyDataObject *self, PyObject *arfs, PyObject *kwds)
{
  return 0;
}

#ifdef ENABLE_LIBFIVE
PyObject *python_lv_void_int(PyObject *self, PyObject *args, PyObject *kwargs,libfive::Tree *t)
{
  std::vector<libfive::Tree *> vec;
  vec.push_back(t);
  return PyDataObjectFromTree(&PyDataType, vec);
}


PyObject *python_lv_un_int(PyObject *self, PyObject *args, PyObject *kwargs,libfive::Opcode::Opcode op)
{
  char *kwlist[] = {"arg",  NULL};
  PyObject *arg = NULL;

  if (!PyArg_ParseTupleAndKeywords(args, kwargs, "O", kwlist, &arg)) return NULL;

  std::vector<libfive::Tree *> tv = PyDataObjectToTree(arg);
  std::vector<libfive::Tree *> res;
  for(int i=0;i<tv.size();i++) {
	  libfive::Tree *sub = new libfive::Tree(libfive::Tree::unary(op, *(tv[i])));
    res.push_back(sub);
  }
  return PyDataObjectFromTree(&PyDataType, res);
}

PyObject *python_lv_bin_int(PyObject *self, PyObject *args, PyObject *kwargs,libfive::Opcode::Opcode op)
{
#if 1
  char *kwlist[] = {"arg1","arg2",  NULL};
  PyObject *arg1 = NULL;
  PyObject *arg2 = NULL;

  if (!PyArg_ParseTupleAndKeywords(args, kwargs, "OO", kwlist,
                                   &arg1,
                                   &arg2
				   )) return NULL;

  std::vector<libfive::Tree *> a1 = PyDataObjectToTree(arg1);
  std::vector<libfive::Tree *> a2 = PyDataObjectToTree(arg2);
  std::vector<libfive::Tree *> res;
  if(a1.size() == a2.size()) {
    for(int i=0;i<a1.size();i++) {		 
      res.push_back(new libfive::Tree(libfive::Tree::binary(op, *a1[i],*a2[i])));
    }
  }
  else if(a2.size() == 1) {
    for(int i=0;i<a1.size();i++) {		 
      res.push_back(new libfive::Tree(libfive::Tree::binary(op, *a1[i],*a2[0])));
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
  Tree res = PyDataObjectToTree(obj);
  for(i=1;i<PyTuple_Size(args);i++)
  {
  	obj= PyTuple_GetItem(args, i);
  	//Py_INCREF(obj);
  	Tree tmp = PyDataObjectToTree(obj);
  	Tree res = libfive_tree_binary(op, res,tmp);
  }
#endif
  return PyDataObjectFromTree(&PyDataType, res);
}

PyObject *python_lv_unop_int(PyObject *arg, libfive::Opcode::Opcode op)
{
  std::vector<libfive::Tree *>t = PyDataObjectToTree(arg);
  std::vector<libfive::Tree *> res;
  for(int i=0;i<t.size();i++) {
    res.push_back( new libfive::Tree(libfive::Tree::unary(op, *(t[i]))));
  }  
  return PyDataObjectFromTree(&PyDataType, res);
}

PyObject *python_lv_binop_int(PyObject *arg1, PyObject *arg2, libfive::Opcode::Opcode op)
{
  std::vector<libfive::Tree *> t1 = PyDataObjectToTree(arg1);
  std::vector<libfive::Tree *> t2 = PyDataObjectToTree(arg2);

  std::vector<libfive::Tree *> res ;
  if(t1.size() == t2.size()) {
    for(int i=0;i<t1.size();i++) {		 
      res.push_back(new libfive::Tree(libfive::Tree::binary(op, *(t1[i]),*(t2[i]))));
    }
  }
  else if(t2.size() == 1) {
    for(int i=0;i<t1.size();i++) {		 
      res.push_back(new libfive::Tree(libfive::Tree::binary(op, *(t1[i]),*(t2[0]))));
    }
  } else { 
    printf("Cannot handle  bin %d binop %d \n",t1.size(), t2.size());
    return Py_None;		    
  }

  return PyDataObjectFromTree(&PyDataType, res);
}

libfive::Tree lv_x = libfive::Tree::X();
libfive::Tree lv_y = libfive::Tree::Y();
libfive::Tree lv_z = libfive::Tree::Z();
PyObject *python_lv_x(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_void_int(self, args, kwargs,&lv_x); }
PyObject *python_lv_y(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_void_int(self, args, kwargs,&lv_y); }
PyObject *python_lv_z(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_void_int(self, args, kwargs,&lv_z); }
PyObject *python_lv_sqrt(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,libfive::Opcode::OP_SQRT); }
PyObject *python_lv_square(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,libfive::Opcode::OP_SQUARE); }
PyObject *python_lv_abs(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,libfive::Opcode::OP_ABS); }
PyObject *python_lv_max(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_bin_int(self, args, kwargs,libfive::Opcode::OP_MAX); }
PyObject *python_lv_min(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_bin_int(self, args, kwargs,libfive::Opcode::OP_MIN); }
PyObject *python_lv_pow(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_bin_int(self, args, kwargs,libfive::Opcode::OP_POW); }
PyObject *python_lv_comp(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_bin_int(self, args, kwargs,libfive::Opcode::OP_COMPARE); }
PyObject *python_lv_atan2(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_bin_int(self, args, kwargs,libfive::Opcode::OP_ATAN2); }

PyObject *python_lv_sin(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,libfive::Opcode::OP_SIN); }
PyObject *python_lv_cos(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,libfive::Opcode::OP_COS); }
PyObject *python_lv_tan(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,libfive::Opcode::OP_TAN); }
PyObject *python_lv_asin(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,libfive::Opcode::OP_ASIN); }
PyObject *python_lv_acos(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,libfive::Opcode::OP_ACOS); }
PyObject *python_lv_atan(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,libfive::Opcode::OP_ATAN); }
PyObject *python_lv_exp(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,libfive::Opcode::OP_EXP); }
PyObject *python_lv_log(PyObject *self, PyObject *args, PyObject *kwargs) { return python_lv_un_int(self, args, kwargs,libfive::Opcode::OP_LOG); }

PyObject *python_lv_print(PyObject *self, PyObject *args, PyObject *kwargs)
{
  char *kwlist[] = {"arg",  NULL};
  PyObject *arg = NULL;

  if (!PyArg_ParseTupleAndKeywords(args, kwargs, "O!", kwlist, &PyDataType, &arg)) return NULL;
  std::vector<libfive::Tree *> tv = PyDataObjectToTree(arg);
  for(int i=0;i<tv.size();i++){
//    printf("tree %d: %s\n",i,libfive_tree_print(tv[i]));
  }
  return Py_None;
}

#endif

static PyMethodDef PyDataFunctions[] = {
#ifdef ENABLE_LIBFIVE	
  {"x", (PyCFunction) python_lv_x, METH_VARARGS | METH_KEYWORDS, "Get X."},
  {"y", (PyCFunction) python_lv_y, METH_VARARGS | METH_KEYWORDS, "Get Y."},
  {"z", (PyCFunction) python_lv_z, METH_VARARGS | METH_KEYWORDS, "Get Z."},
  {"sqrt", (PyCFunction) python_lv_sqrt, METH_VARARGS | METH_KEYWORDS, "Square Root"},
  {"square", (PyCFunction) python_lv_square, METH_VARARGS | METH_KEYWORDS, "Square"},
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
  {"atan2", (PyCFunction) python_lv_atan2, METH_VARARGS | METH_KEYWORDS, "Atan2"},
  {"print", (PyCFunction) python_lv_print, METH_VARARGS | METH_KEYWORDS, "Print"},
#endif  
  {NULL, NULL, 0, NULL}
};

#ifdef ENABLE_LIBFIVE
PyObject *python_lv_add(PyObject *arg1, PyObject *arg2) { return python_lv_binop_int(arg1, arg2,  libfive::Opcode::OP_ADD); }
PyObject *python_lv_substract(PyObject *arg1, PyObject *arg2) { return python_lv_binop_int(arg1, arg2,  libfive::Opcode::OP_SUB); }
PyObject *python_lv_multiply(PyObject *arg1, PyObject *arg2) { return python_lv_binop_int(arg1, arg2,  libfive::Opcode::OP_MUL); }
PyObject *python_lv_remainder(PyObject *arg1, PyObject *arg2) { return python_lv_binop_int(arg1, arg2,  libfive::Opcode::OP_MOD); }
PyObject *python_lv_divide(PyObject *arg1, PyObject *arg2) { return python_lv_binop_int(arg1, arg2,  libfive::Opcode::OP_DIV); }
PyObject *python_lv_negate(PyObject *arg) { return python_lv_unop_int(arg, libfive::Opcode::OP_NEG); }
#else
PyObject *python_lv_add(PyObject *arg1, PyObject *arg2) { return Py_None; }
PyObject *python_lv_substract(PyObject *arg1, PyObject *arg2) { return Py_None; }
PyObject *python_lv_multiply(PyObject *arg1, PyObject *arg2) { return  Py_None; }
PyObject *python_lv_remainder(PyObject *arg1, PyObject *arg2) { return Py_None; }
PyObject *python_lv_divide(PyObject *arg1, PyObject *arg2) { return  Py_None; }
PyObject *python_lv_negate(PyObject *arg) { return  Py_None; }
#endif

extern PyTypeObject PyOpenSCADType;
Value python_convertresult(PyObject *arg, int &error);

PyObject *PyDataObject_call(PyObject *self, PyObject *args, PyObject *kwargs)
{
//  printf("Call %d\n",PyTuple_Size(args));
  AssignmentList pargs;
  int error;
  for(int i=0;i<PyTuple_Size(args);i++) {
    Value val = python_convertresult(PyTuple_GetItem(args,i),error);	  
    std::shared_ptr<Literal> lit = std::make_shared<Literal>(std::move(val), Location::NONE);
    std::shared_ptr<Assignment> ass  = std::make_shared<Assignment>(std::string(""),lit);
    pargs.push_back(ass);
  }

  boost::optional<InstantiableModule> mod =PyDataObjectToModule(self);
  std::string instance_name; 
  ModuleInstantiation *instance = new ModuleInstantiation(instance_name,pargs, Location::NONE);

  EvaluationSession session{"."};
  ContextHandle<BuiltinContext> c_handle{Context::create<BuiltinContext>(&session)};
  std::shared_ptr<const BuiltinContext> cxt = *c_handle;

  auto resultnode = mod->module->instantiate(cxt, instance, cxt);
  return PyOpenSCADObjectFromNode(&PyOpenSCADType, resultnode);
}

PyNumberMethods PyDataNumbers =
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

PyTypeObject PyDataType = {
    PyVarObject_HEAD_INIT(NULL, 0)
    "PyData",             			/* tp_name */
    sizeof(PyDataObject), 			/* tp_basicsize */
    0,                         			/* tp_itemsize */
    PyDataObject_dealloc,			/* tp_dealloc */
    0,                         			/* tp_print */
    0,                         			/* tp_getattr */
    0,                         			/* tp_setattr */
    0,                         			/* tp_reserved */
    0,                         			/* tp_repr */
    &PyDataNumbers, 				/* tp_as_number */
    0,                         			/* tp_as_sequence */
    0,		        			/* tp_as_mapping */
    0,                         			/* tp_hash  */
    PyDataObject_call,         			/* tp_call */
    0,                         			/* tp_str */
    0,                         			/* tp_getattro */
    0,                         			/* tp_setattro */
    0,                         			/* tp_as_buffer */
    Py_TPFLAGS_DEFAULT|Py_TPFLAGS_HAVE_VERSION_TAG|Py_TPFLAGS_BASETYPE, 	/* tp_flags */
    0,			          		/* tp_doc */
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
    (initproc) PyDataInit,      		/* tp_init */
    0,                         			/* tp_alloc */
    PyDataObject_new,                	/* tp_new */
};


static PyModuleDef DataModule = {
  PyModuleDef_HEAD_INIT,
  "Data",
  "Example module that creates an extension type.",
  -1,
  PyDataFunctions,
  NULL, NULL, NULL, NULL
};

PyMODINIT_FUNC PyInit_PyData(void)
{
  PyObject *m;
  if (PyType_Ready(&PyDataType) < 0) return NULL;
  m = PyModule_Create(&DataModule);
  if (m == NULL) return NULL;

  Py_INCREF(&PyDataType);
  PyModule_AddObject(m, "openscad", (PyObject *)&PyDataType);
  return m;
}

PyObject *PyInit_data(void)
{
  return PyModule_Create(&DataModule);
}

