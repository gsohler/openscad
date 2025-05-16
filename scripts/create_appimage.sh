#! /bin/sh
rm -rf AppDir
cd build
make install DESTDIR=/AppDir
cd ..
export PATH=/appimage/usr/local/bin:$PATH
export EXTRA_QT_PLUGINS=svg
linuxdeploy --plugin qt --output appimage --appdir AppDir

