#! /bin/sh

# create tempdir
mkdir -p tmp
cd tmp
rm -rf *

#unpack
ar x ../python_mingw/ucrt64/lib/libpython3.11.dll.a
#patch
bspatch libpython3_11_dll_d001667.o libpython3_11_dll_d001667.o.tmp ../libpython3_11_dll_d001667.o.diff
mv libpython3_11_dll_d001667.o.tmp libpython3_11_dll_d001667.o

#pack again
ar -rc ../python_mingw/ucrt64/lib/libpython3.11.dll.a *.o


 
