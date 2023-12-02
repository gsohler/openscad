/*
 *  OpenSCAD (www.openscad.org)
 *  Copyright (C) 2009-2011 Clifford Wolf <clifford@clifford.at> and
 *                          Marius Kintel <marius@kintel.net>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  As a special exception, you have permission to link this program
 *  with the CGAL library and distribute executables, as long as you
 *  follow the requirements of the GNU GPL in regard to all of the
 *  software in the executable aside from CGAL.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
#include <Python.h>
#include "pyopenscad.h"
#include "CsgOpNode.h"
#include "Value.h"
#include "Expression.h"
#include "PlatformUtils.h"
#include <Context.h>

// #define HAVE_PYTHON_YIELD
static PyObject *PyInit_openscad(void);

// https://docs.python.org/3.10/extending/newtypes.html 

PyObject *pythonInitDict = nullptr;
PyObject *pythonMainModule = nullptr ;
std::list<std::string> pythonInventory;
bool python_active;  /* if python is actually used during evaluation */
bool python_trusted; /* global Python trust flag */
#include "PlatformUtils.h"
bool pythonMainModuleInitialized = false;
bool pythonRuntimeInitialized = false;

void PyOpenSCADObject_dealloc(PyOpenSCADObject *self)
{
  Py_XDECREF(self->dict);
//  Py_TYPE(self)->tp_free((PyObject *)self);
}

PyObject *PyOpenSCADObject_alloc(PyTypeObject *cls, Py_ssize_t nitems)
{
  return PyType_GenericAlloc(cls, nitems);
}

/*
 *  allocates a new PyOpenSCAD Object including its internal dictionary
 */

static PyObject *PyOpenSCADObject_new(PyTypeObject *type, PyObject *args,  PyObject *kwds)
{
  (void)args;
  (void)kwds;
  PyObject * empty_tuple;
  PyOpenSCADObject *self;
  empty_tuple = PyTuple_New(0);
  Py_XINCREF(empty_tuple);
  self = (PyOpenSCADObject *) PyBaseObject_Type.tp_new(type, empty_tuple, 0);
  Py_XDECREF(empty_tuple);
  self->node = nullptr;
  self->dict = PyDict_New();
  Py_XINCREF(self->dict);
  Py_XINCREF(self);
  return (PyObject *)self;
}

/*
 *  allocates a new PyOpenSCAD to store an existing OpenSCAD Abstract Node
 */

PyObject *PyOpenSCADObjectFromNode(PyTypeObject *type, const std::shared_ptr<AbstractNode> &node)
{
  PyOpenSCADObject *self;
  self = (PyOpenSCADObject *)  type->tp_alloc(type, 0);
  if (self != NULL) {
    self->node = node;
    self->dict = PyDict_New();
    Py_XINCREF(self->dict);
    Py_XINCREF(self);
    return (PyObject *)self;
  }
  return nullptr;
}

/*
 *  parses either a PyOpenSCAD Object or an List of PyOpenScad Object and adds it to the list of supplied children, returns 1 on success
 */

int python_more_obj(std::vector<std::shared_ptr<AbstractNode>>& children, PyObject *more_obj) {
  int i, n;
  PyObject *obj;
  std::shared_ptr<AbstractNode> child;
  if (PyList_Check(more_obj)) {
    n = PyList_Size(more_obj);
    for (i = 0; i < n; i++) {
      obj = PyList_GetItem(more_obj, i);
      child = PyOpenSCADObjectToNode(obj);
      children.push_back(child);
    }
  } else if (Py_TYPE(more_obj) == &PyOpenSCADType) {
    child = PyOpenSCADObjectToNode(more_obj);
    children.push_back(child);
  } else return 1;
  return 0;
}

/*
 *  extracts Absrtract Node from PyOpenSCAD Object
 */

std::shared_ptr<AbstractNode> PyOpenSCADObjectToNode(PyObject *obj)
{
  std::shared_ptr<AbstractNode> result = ((PyOpenSCADObject *) obj)->node;
  Py_XDECREF(obj); 
  return result;
}


/*
 * same as  python_more_obj but always returns only one AbstractNode by creating an UNION operation
 */

std::shared_ptr<AbstractNode> PyOpenSCADObjectToNodeMulti(PyObject *objs)
{
  std::shared_ptr<AbstractNode> result;
  if (Py_TYPE(objs) == &PyOpenSCADType) {
    result = ((PyOpenSCADObject *) objs)->node;
  } else if (PyList_Check(objs)) {

    DECLARE_INSTANCE
    auto node = std::make_shared<CsgOpNode>(instance, OpenSCADOperator::UNION);

    int n = PyList_Size(objs);
    for (int i = 0; i < n; i++) {
      PyObject *obj = PyList_GetItem(objs, i);
      if(Py_TYPE(obj) ==  &PyOpenSCADType) {
        std::shared_ptr<AbstractNode> child = PyOpenSCADObjectToNode(obj);
        node->children.push_back(child);
        Py_XDECREF(obj);
      } else return nullptr;
    }
    result=node;
  } else result=nullptr;
//  Py_XDECREF(objs);
  return result;
}

/*
 * converts a python obejct into an integer by all means
 */

int python_numberval(PyObject *number, double *result)
{
  if(number == Py_False) return 1;	
  if(number == Py_True) return 1;	
  if(number == Py_None) return 1;	
  if (PyFloat_Check(number)) {
    *result = PyFloat_AsDouble(number);
    return 0;
  }
  if (PyLong_Check(number)) {
    *result = PyLong_AsLong(number);
    return 0;
  }
  return 1;
}

/*
 * Tries to extract an 3D vector out of a python list
 */

int python_vectorval(PyObject *vec, double *x, double *y, double *z, double *w)
{
  *x = 1.0;
  *y = 1.0;
  if(w != NULL ) *w = 0;
  if (PyList_Check(vec)) {
    if (PyList_Size(vec) >= 1) {
      if (python_numberval(PyList_GetItem(vec, 0), x)) return 1;
    }
    if (PyList_Size(vec) >= 2) {
      if (python_numberval(PyList_GetItem(vec, 1), y)) return 1;
    }
    if (PyList_Check(vec) && PyList_Size(vec) >= 3) {
      if (python_numberval(PyList_GetItem(vec, 2), z)) return 1;
    }
    if (PyList_Check(vec) && PyList_Size(vec) >= 4 && w != NULL) {
      if (python_numberval(PyList_GetItem(vec, 3), w)) return 1;
    }
    return 0;
  }
  if (!python_numberval(vec, x)) {
    *y = *x;
    *z = *x;
    if(w != NULL) *w = *x;
    return 0;
  }
  return 1;
}

/*
 * Helper function to extract actual values for fn, fa and fs
 */

void get_fnas(double& fn, double& fa, double& fs) {
  PyObject *mainModule = PyImport_AddModule("__main__");
  if (mainModule == nullptr) return;
  PyObject *varFn = PyObject_GetAttrString(mainModule, "fn");
  PyObject *varFa = PyObject_GetAttrString(mainModule, "fa");
  PyObject *varFs = PyObject_GetAttrString(mainModule, "fs");
  if (varFn != nullptr) fn = PyFloat_AsDouble(varFn);
  if (varFa != nullptr) fa = PyFloat_AsDouble(varFa);
  if (varFs != nullptr) fs = PyFloat_AsDouble(varFs);
}

/*
 * Helper function to offer OO functions without having to rewrite all funcions in 2 variants(cascading functions)
 */

PyObject *python_oo_args(PyObject *self, PyObject *args)
{
  int i;
  PyObject *item;
  int n = PyTuple_Size(args);
  PyObject *new_args = PyTuple_New(n + 1);
//	Py_INCREF(new_args); // dont decref either,  dont know why its not working

  Py_INCREF(self);
  int ret = PyTuple_SetItem(new_args, 0, self);
  item = PyTuple_GetItem(new_args, 0);

  for (i = 0; i < PyTuple_Size(args); i++) {
    item = PyTuple_GetItem(args, i);
    Py_INCREF(item);
    PyTuple_SetItem(new_args, i + 1, item);
  }
  return new_args;
}


/*
 * Type specific init function. nothing special here
 */

static int PyOpenSCADInit(PyOpenSCADObject *self, PyObject *arfs, PyObject *kwds)
{
  (void)self;
  (void)arfs;
  (void)kwds;
  return 0;
}

std::shared_ptr<AbstractNode> python_result_node = NULL;
Outline2d python_getprofile(void *v_cbfunc, int fn, double arg)
{
	PyObject *cbfunc = (PyObject *) v_cbfunc;
	Outline2d result;
	if(pythonInitDict == NULL)  initPython();
	PyObject* args = PyTuple_Pack(1,PyFloat_FromDouble(arg));
	PyObject* polygon = PyObject_CallObject(cbfunc, args);
	if(polygon == NULL) { // TODO fix
		for(unsigned int i=0;i < fn;i++) {
			double ang=360.0*(i/(double) fn);
			PyObject* args = PyTuple_Pack(2,PyFloat_FromDouble(arg),PyFloat_FromDouble(ang));
			Py_XINCREF(args);
			PyObject* pypt = PyObject_CallObject(cbfunc, args);
			double r=PyFloat_AsDouble(pypt);
			if(r < 0) r=-r;  // TODO who the hell knows, why this is needed
			double ang1=ang*3.1415/180.0;
			double x=r*cos(ang1);
			double y=r*sin(ang1);
			result.vertices.push_back(Vector2d(x,y));
		}
	} else if(polygon && PyList_Check(polygon)) {
		unsigned int n=PyList_Size(polygon);
		for(unsigned int i=0;i < n;i++) {
			PyObject *pypt = PyList_GetItem(polygon, i);
			if(PyList_Check(pypt) && PyList_Size(pypt) == 2) {
				double x=PyFloat_AsDouble(PyList_GetItem(pypt, 0));
				double y=PyFloat_AsDouble(PyList_GetItem(pypt, 1));
				result.vertices.push_back(Vector2d(x,y));
			}
		}
	}
	if(result.vertices.size() < 3)
	{
		Outline2d err;
		err.vertices.push_back(Vector2d(0,0));
		err.vertices.push_back(Vector2d(10,0));
		err.vertices.push_back(Vector2d(10,10));
		return err;
	}
	return result;
}

double python_doublefunc(void *v_cbfunc, double arg)
{
	PyObject *cbfunc = (PyObject *) v_cbfunc;
	double result=0;
	PyObject* args = PyTuple_Pack(1,PyFloat_FromDouble(arg));
	PyObject* funcresult = PyObject_CallObject(cbfunc, args);
	if(funcresult)
		result=PyFloat_AsDouble(funcresult);
	return result;
}

/*
 * Try to call a python function by name using OpenSCAD module childs and OpenSCAD function arguments: argument order is childs, arguments
 */

PyObject *python_callfunction(const std::shared_ptr<const Context> &cxt , const std::string &name, const std::vector<std::shared_ptr<Assignment> > &op_args, const char *&errorstr)
{
	PyObject *pFunc = nullptr;
	if(!pythonMainModule){
		return nullptr;
	}
	PyObject *maindict = PyModule_GetDict(pythonMainModule);

	// search the function in all modules
	PyObject *key, *value;
	Py_ssize_t pos = 0;

	while (PyDict_Next(maindict, &pos, &key, &value)) {
		PyObject *module = PyObject_GetAttrString(pythonMainModule, PyUnicode_AsUTF8(key));
		if(module == nullptr) continue;
		PyObject *moduledict = PyModule_GetDict(module);
		if(moduledict == nullptr) continue;
        	pFunc = PyDict_GetItemString(moduledict, name.c_str());
		if(pFunc == nullptr) continue;
		break;
	}
	if (!pFunc) {
		return nullptr;
	}
	if (!PyCallable_Check(pFunc)) {
		return nullptr;
	}
	
	PyObject *args = PyTuple_New(op_args.size());
	for(int i=0;i<op_args.size();i++)
	{
		Assignment *op_arg=op_args[i].get();
		shared_ptr<Expression> expr=op_arg->getExpr();
		Value val = expr.get()->evaluate(cxt);
		switch(val.type())
		{
			case Value::Type::NUMBER:
				PyTuple_SetItem(args, i, PyFloat_FromDouble(val.toDouble()));
				break;
			case Value::Type::STRING:
				PyTuple_SetItem(args, i, PyUnicode_FromString(val.toString().c_str()));
				break;
//TODO  more types RANGE, VECTOR, OBEJCT, FUNCTION
			default:
				PyTuple_SetItem(args, i, PyLong_FromLong(-1));
				break;
		}
	}
	PyObject* funcresult = PyObject_CallObject(pFunc, args);

	if(funcresult == nullptr) {
		PyObject *pyExcType;
		PyObject *pyExcValue;
		PyObject *pyExcTraceback;
		PyErr_Fetch(&pyExcType, &pyExcValue, &pyExcTraceback);
		PyErr_NormalizeException(&pyExcType, &pyExcValue, &pyExcTraceback);

		PyObject* str_exc_value = PyObject_Repr(pyExcValue);
		PyObject* pyExcValueStr = PyUnicode_AsEncodedString(str_exc_value, "utf-8", "~");
		errorstr =  PyBytes_AS_STRING(pyExcValueStr);
		Py_XDECREF(pyExcType);
		Py_XDECREF(pyExcValue);
		Py_XDECREF(pyExcTraceback);
		return nullptr;
	}
	return funcresult;
}

/*
 * Actually trying use python to evaluate a OpenSCAD Module
 */

std::shared_ptr<AbstractNode> python_modulefunc(const ModuleInstantiation *op_module,const std::shared_ptr<const Context> &cxt, int *modulefound)
{
	*modulefound=0;
	std::shared_ptr<AbstractNode> result=nullptr;
	const char *errorstr = nullptr;
	do {
		PyObject *funcresult = python_callfunction(cxt,op_module->name(),op_module->arguments, errorstr);
		if (errorstr != NULL){
			PyErr_SetString(PyExc_TypeError, errorstr);
			return NULL;
		}
		*modulefound=1;
		if(funcresult == NULL) return NULL;

		if(funcresult->ob_type == &PyOpenSCADType) result=PyOpenSCADObjectToNode(funcresult);
		else {
			// ignore wrong type. just output valid empty geometry
//			LOG(message_group::Warning, Location::NONE, cxt->documentRoot(), "Python function result is not a solid.");
//			break;
		}
	} while(0);
	return result;
}

/*
 * Converting a python result to an openscad result. extra function required as it might call itself hierarchically
 */

Value python_convertresult(PyObject *arg)
{
	if(arg == nullptr) return Value::undefined.clone();
	if(PyList_Check(arg)) {
		VectorType vec(nullptr);
		for(int i=0;i<PyList_Size(arg);i++) {
			PyObject *item=PyList_GetItem(arg,i);
			vec.emplace_back(python_convertresult(item));
		}
		return std::move(vec);
	} else if(PyFloat_Check(arg)) { return { PyFloat_AsDouble(arg) }; }
	else if(PyUnicode_Check(arg)) {
		PyObject* repr = PyObject_Repr(arg);
		PyObject* strobj = PyUnicode_AsEncodedString(repr, "utf-8", "~");
		const char *chars =  PyBytes_AS_STRING(strobj);
		return { std::string(chars) } ;
	} else {
		PyErr_SetString(PyExc_TypeError, "Unsupported function result\n");
		return Value::undefined.clone();
	}
}

/*
 * Actually trying use python to evaluate a OpenSCAD Function
 */

Value python_functionfunc(const FunctionCall *call,const std::shared_ptr<const Context> &cxt  )
{
	const char *errorstr = nullptr;
	PyObject *funcresult = python_callfunction(cxt,call->name, call->arguments, errorstr);
	if (errorstr != nullptr)
	{
		PyErr_SetString(PyExc_TypeError, errorstr);
		return Value::undefined.clone();
	}
	if(funcresult == nullptr) return Value::undefined.clone();

	return  python_convertresult(funcresult);
}

#ifdef ENABLE_LIBFIVE
extern PyObject *PyInit_libfive(void);
PyMODINIT_FUNC PyInit_PyLibFive(void);
#endif
/*
 * Main python evaluation entry
 */

#ifdef HAVE_PYTHON_YIELD
std::vector<PyObject *> python_orphan_objs;
extern "C" {
	void set_object_callback(void (*object_capture_callback)(PyObject *));
}
void openscad_object_callback(PyObject *obj) {
	if(obj->ob_type == &PyOpenSCADType) {
  		Py_INCREF(obj);
		python_orphan_objs.push_back(obj);
	}
}
#endif
void initPython(void)
{
  if(pythonInitDict) { /* If already initialized, undo to reinitialize after */
    PyObject *key, *value;
    Py_ssize_t pos = 0;
    PyObject *maindict = PyModule_GetDict(pythonMainModule);
    while (PyDict_Next(maindict, &pos, &key, &value)) {
      PyObject* key1 = PyUnicode_AsEncodedString(key, "utf-8", "~");
      const char *key_str =  PyBytes_AS_STRING(key1);
      if(key_str == NULL) continue;
      if (std::find(std::begin(pythonInventory), std::end(pythonInventory), key_str) == std::end(pythonInventory))
      {
        PyDict_DelItemString(maindict, key_str); // TODO does not work!
      }
    }
  } else {
#ifdef HAVE_PYTHON_YIELD
    set_object_callback(openscad_object_callback);
#endif
    char run_str[200];
    PyImport_AppendInittab("openscad", &PyInit_openscad);
#ifdef ENABLE_LIBFIVE	    
    PyImport_AppendInittab("libfive", &PyInit_libfive);
#endif	    
    PyConfig config;
    PyConfig_InitPythonConfig(&config);
    char libdir[256];
    snprintf(libdir, 256, "%s/../libraries/python/",PlatformUtils::applicationPath().c_str()); /* add libraries/python to python search path */
    PyConfig_SetBytesString(&config, &config.pythonpath_env, libdir);
    Py_InitializeFromConfig(&config);
    PyConfig_Clear(&config);

    pythonMainModule =  PyImport_AddModule("__main__");
    pythonInitDict = PyModule_GetDict(pythonMainModule);
    PyInit_PyOpenSCAD();
#ifdef ENABLE_LIBFIVE	    
    PyInit_PyLibFive();
#endif	    
    PyRun_String("from builtins import *\nfrom openscad import *\n", Py_file_input, pythonInitDict, pythonInitDict);
    sprintf(run_str,"fa=12.0\nfn=0.0\nfs=2.0\nt=%g",time);
    PyRun_String(run_str, Py_file_input, pythonInitDict, pythonInitDict);

      // find base variables
    PyObject *key, *value;
    Py_ssize_t pos = 0;
    PyObject *maindict = PyModule_GetDict(pythonMainModule);
    while (PyDict_Next(maindict, &pos, &key, &value)) {
      PyObject* key1 = PyUnicode_AsEncodedString(key, "utf-8", "~");
      const char *key_str =  PyBytes_AS_STRING(key1);
      if(key_str == NULL) continue;
      pythonInventory.push_back(key_str);
    }
  }
}

void finishPython(void)
{
      // annotation "Parameter" missing
//      if (Py_FinalizeEx() < 0) {
//        exit(120);
//      }
//      pythonInitDict=NULL;
#ifdef HAVE_PYTHON_YIELD
      set_object_callback(NULL);
      if(python_result_node == nullptr) {
        if(python_orphan_objs.size() == 1) {
  	  python_result_node = PyOpenSCADObjectToNode(python_orphan_objs[0]);
        } else if(python_orphan_objs.size() > 1) {
          DECLARE_INSTANCE
	  auto node = std::make_shared<CsgOpNode>(instance, OpenSCADOperator::UNION);
          int n = python_orphan_objs.size();
          for (int i = 0; i < n; i++) {
            std::shared_ptr<AbstractNode> child = PyOpenSCADObjectToNode(python_orphan_objs[i]);
            node->children.push_back(child);
          } 
          python_result_node=node;
        } 
      }
#endif
}

std::string evaluatePython(const std::string & code, double time,AssignmentList &assignments)
{
  std::string error;
  python_result_node = nullptr;
  PyObject *pyExcType = nullptr;
  PyObject *pyExcValue = nullptr;
  PyObject *pyExcTraceback = nullptr;
  /* special python code to catch errors from stdout and stderr and make them available in OpenSCAD console */
  const char *python_init_code="\
import sys\n\
class OutputCatcher:\n\
   def __init__(self):\n\
      self.data = ''\n\
   def write(self, stuff):\n\
      self.data = self.data + stuff\n\
   def flush(self):\n\
      pass\n\
catcher_out = OutputCatcher()\n\
catcher_err = OutputCatcher()\n\
stdout_bak=sys.stdout\n\
stderr_bak=sys.stderr\n\
sys.stdout = catcher_out\n\
sys.stderr = catcher_err\n\
";
  const char *python_exit_code="\
sys.stdout = stdout_bak\n\
sys.stderr = stderr_bak\n\
";

    PyRun_SimpleString(python_init_code);
#ifdef HAVE_PYTHON_YIELD
    for(auto obj : python_orphan_objs) {
        Py_DECREF(obj);
    }
    python_orphan_objs.clear();
#endif
    PyObject *result = PyRun_String(code.c_str(), Py_file_input, pythonInitDict, pythonInitDict); /* actual code is run here */

    PyObject *key, *value;
    Py_ssize_t pos = 0;
    PyObject *pFunc;

    PyObject *maindict = PyModule_GetDict(pythonMainModule);
    assignments.clear();
    while (PyDict_Next(maindict, &pos, &key, &value)) {
      PyObject* key1 = PyUnicode_AsEncodedString(key, "utf-8", "~");
      const char *key_str =  PyBytes_AS_STRING(key1);
      if(key_str == nullptr) continue;
      if(strcmp(key_str,"fn") == 0) continue;
      if(strcmp(key_str,"fa") == 0) continue;
      if(strcmp(key_str,"fs") == 0) continue;
      if(strcmp(key_str,"t") == 0) continue;
      if(strcmp(key_str,"__name__") == 0) continue;
      // annotation "Parameter" missing
      std::shared_ptr<Literal> lit;
      bool found=false;
      if(value == Py_True) {
        lit = std::make_shared<Literal>(true,Location::NONE);
        found=true;
      } else if(value == Py_False) {
        lit = std::make_shared<Literal>(false,Location::NONE);
        found=true;
      } else if(PyFloat_Check(value)) {
        lit  = std::make_shared<Literal>(PyFloat_AsDouble(value),Location::NONE);
        found=true;
      }
      else if(PyLong_Check(value)){
        lit = std::make_shared<Literal>(PyLong_AsLong(value)*1.0,Location::NONE);
        found=true;
      }
      else if(PyUnicode_Check(value)){
        PyObject* value1 = PyUnicode_AsEncodedString(value, "utf-8", "~");
        const char *value_str =  PyBytes_AS_STRING(value1);
        lit = std::make_shared<Literal>(value_str,Location::NONE);
        found=true;
      }
      if(found == true) {
        auto assignment = std::make_shared<Assignment>(key_str,lit);
        assignments.push_back(assignment);
      }
    }

    if(result  == nullptr) PyErr_Print();
    PyRun_SimpleString(python_exit_code);

    for(int i=0;i<2;i++)
    {
      PyObject* catcher = PyObject_GetAttrString(pythonMainModule, i==1?"catcher_err":"catcher_out");
      PyObject* command_output = PyObject_GetAttrString(catcher, "data");
      PyObject* command_output_value = PyUnicode_AsEncodedString(command_output, "utf-8", "~");
      const char *command_output_bytes =  PyBytes_AS_STRING(command_output_value);
      if(command_output_bytes == nullptr || *command_output_bytes == '\0') continue;
      if(i ==1) error += command_output_bytes; /* output to console */
      else LOG(command_output_bytes); /* error to LOG */
    }

    PyErr_Fetch(&pyExcType, &pyExcValue, &pyExcTraceback); /* extract actual python stack trace in case of an expception and return the error string to the caller */
//    PyErr_NormalizeException(&pyExcType, &pyExcValue, &pyExcTraceback);
    PyObject* str_exc_value = PyObject_Repr(pyExcValue);
    PyObject* pyExcValueStr = PyUnicode_AsEncodedString(str_exc_value, "utf-8", "~");
    if(str_exc_value != nullptr) Py_XDECREF(str_exc_value);
    const char *strExcValue =  PyBytes_AS_STRING(pyExcValueStr);
    if(pyExcValueStr != nullptr) Py_XDECREF(pyExcValueStr);
    if(strExcValue != nullptr && strcmp(strExcValue,"<NULL>") != 0) error += strExcValue;
    if(pyExcTraceback != nullptr) {
      auto *tb_o = (PyTracebackObject *)pyExcTraceback;
      int line_num = tb_o->tb_lineno;
      error += " in line ";
      error += std::to_string(line_num);
      Py_XDECREF(pyExcTraceback);
    }

    if(pyExcType != nullptr) Py_XDECREF(pyExcType);
    if(pyExcValue != nullptr) Py_XDECREF(pyExcValue);
    return error;
}
/*
 * the magical Python Type descriptor for an OpenSCAD Object. Adding more fields makes the type more powerful
 */

PyTypeObject PyOpenSCADType = {
    PyVarObject_HEAD_INIT(nullptr, 0)
    "PyOpenSCAD",             			/* tp_name */
    sizeof(PyOpenSCADObject), 			/* tp_basicsize */
    0,                         			/* tp_itemsize */
    (destructor) PyOpenSCADObject_dealloc,	/* tp_dealloc */
    0,                         			/* vectorcall_offset */
    nullptr,                         			/* tp_getattr */
    nullptr,                         			/* tp_setattr */
    nullptr,                         			/* tp_as_async */
    nullptr,                         			/* tp_repr */
    &PyOpenSCADNumbers,        			/* tp_as_number */
    nullptr,                         			/* tp_as_sequence */
    &PyOpenSCADMapping,        			/* tp_as_mapping */
    nullptr,                         			/* tp_hash  */
    nullptr,                         			/* tp_call */
    nullptr,                         			/* tp_str */
    nullptr,                         			/* tp_getattro */
    nullptr,                         			/* tp_setattro */
    nullptr,                         			/* tp_as_buffer */
    Py_TPFLAGS_DEFAULT | Py_TPFLAGS_BASETYPE,	/* tp_flags */
    "PyOpenSCAD Object",          		/* tp_doc */
    nullptr,                         			/* tp_traverse */
    nullptr,                         			/* tp_clear */
    nullptr,                         			/* tp_richcompare */
    0,                         			/* tp_weaklistoffset */
    nullptr,                         			/* tp_iter */
    nullptr,                         			/* tp_iternext */
    PyOpenSCADMethods,             		/* tp_methods */
    nullptr,             				/* tp_members */
    nullptr,                         			/* tp_getset */
    nullptr,                         			/* tp_base */
    nullptr,                         			/* tp_dict */
    nullptr,                         			/* tp_descr_get */
    nullptr,                         			/* tp_descr_set */
    0,                         			/* tp_dictoffset */
    (initproc) PyOpenSCADInit,      		/* tp_init */
    PyOpenSCADObject_alloc,    			/* tp_alloc */
    PyOpenSCADObject_new,                	/* tp_new */
};



static PyModuleDef OpenSCADModule = {
  PyModuleDef_HEAD_INIT,
  "openscad",
  "OpenSCAD Python Module",
  -1,
  PyOpenSCADFunctions,
  nullptr, nullptr, nullptr, nullptr
};

static PyObject *PyInit_openscad(void)
{
  return PyModule_Create(&OpenSCADModule);
}

PyMODINIT_FUNC PyInit_PyOpenSCAD(void)
{
  PyObject *m;

  if (PyType_Ready(&PyOpenSCADType) < 0) return nullptr;
  m = PyModule_Create(&OpenSCADModule);
  if (m == nullptr) return nullptr;

  Py_INCREF(&PyOpenSCADType);
  PyModule_AddObject(m, "openscad", (PyObject *)&PyOpenSCADType);
  return m;
}

