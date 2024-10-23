#include "io/import.h"
#include "geometry/PolySet.h"
#include "geometry/PolySetBuilder.h"

// https://stepcode.github.io/docs/p21_cpp_example/
//#include <sc_cf.h>
extern void SchemaInit( class Registry & );
#include "STEPfile.h"
#include "sdai.h"
#include <STEPattribute.h>
#include <ExpDict.h>
#include <Registry.h>
#include <errordesc.h>

//#include <ios>
//#include <memory>
//#include <fstream>
//#include <string>
//#include <vector>
//#include <boost/regex.hpp>
//#include <boost/lexical_cast.hpp>
//#include <boost/algorithm/string.hpp>
//#include <boost/algorithm/string/split.hpp>

std::unique_ptr<PolySet> import_step(const std::string& filename, const Location& loc) {
  PolySetBuilder builder;
  printf("Importing %s\n",filename.c_str());

  // the registry contains information about types present in the current schema; SchemaInit 
    // is a function in the schema-specific SDAI library
  Registry  registry( SchemaInit );

  //the InstMgr holds instances that have been created or that have been loaded from a file
  InstMgr   instance_list;

  // STEPfile takes care of reading and writing Part 21 files
  STEPfile  sfile( registry, instance_list, "", false );

  // read a file, using the name from the command line
  sfile.ReadExchangeFile(filename.c_str());

  // check for errors; exit if they are particularly severe
  if( sfile.Error().severity() < SEVERITY_USERMSG ) {
        sfile.Error().PrintContents( cout );
  }
  if ( sfile.Error().severity() <= SEVERITY_INCOMPLETE ) {
        exit(1);
  }

  /**************************************************
  ** do something with the data here
  ***************************************************/

  // write to "file.out", then check for write errors. The write operation overwrites any
  // errors caused by previous operations.
//  sfile.WriteExchangeFile( "file.out" );
//  if( sfile.Error().severity() < SEVERITY_USERMSG ) {
//    sfile.Error().PrintContents( cout );
//  }
  return builder.build();
}
