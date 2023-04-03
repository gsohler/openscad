#pragma once

#include "node.h"
#include "Value.h"
#include <linalg.h>
#include "PolySet.h"

#ifdef ENABLE_PYTHON
#include <Python.h>
#endif
#include <libfive.h>
#include "libfive/oracle/oracle_clause.hpp"
#include "libfive/oracle/oracle_storage.hpp"

PyObject *ifrep(const PolySet *ps);
typedef std::vector<int> intList;

struct CutFace
{
  double a,b,c,d;
} ;

struct CutProgram
{
  double a,b,c,d;
  int posbranch;
  int negbranch;
} ;

class OpenSCADOracle : public libfive::OracleStorage<LIBFIVE_EVAL_ARRAY_SIZE> {
public:
    OpenSCADOracle(int x);
    void evalInterval(libfive::Interval& out) override;
    void evalPoint(float& out, size_t index=0) override;
    void checkAmbiguous( Eigen::Block<Eigen::Array<bool, 1, LIBFIVE_EVAL_ARRAY_SIZE>, 1, Eigen::Dynamic> /* out */) override;

    // Find one derivative with partial differences
    void evalFeatures(boost::container::small_vector<libfive::Feature, 4>& out) override;

protected:
    int x;
};

class OpenSCADOracleClause: public libfive::OracleClause
{
public:
    OpenSCADOracleClause(int x)
        : x(x)
    {
        // Nothing to do here
    }

    std::unique_ptr<libfive::Oracle> getOracle() const override {
        return std::unique_ptr<libfive::Oracle>(new OpenSCADOracle(x));
    }

    std::string name() const override { return "OpenSCADOracleClause"; }

protected:
    int x;
};

class FrepNode : public LeafNode
{
public:
  FrepNode(const ModuleInstantiation *mi) : LeafNode(mi) {}
  std::string toString() const override
  {
    std::ostringstream stream;
    stream << "sdf( " << rand() << ")";
    return  stream.str();
  }
  std::string name() const override { return "sdf"; }
  const Geometry *createGeometry() const override;
 #ifdef ENABLE_PYTHON
  PyObject *expression;
 #endif  
  double x1,y1,z1;
  double x2,y2,z2;
  double res;
};

