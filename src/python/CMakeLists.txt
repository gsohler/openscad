project(python)

add_subdirectory(pybind11)

pybind11_add_module(openscad pymodule.cc)

target_include_directories(openscad PRIVATE
    ".."
    "../.."
    "../core"
    "pybind11/include"
    "../core/customizer"
    "../ext"
    "../ext/json"
    "../ext/lexertl/include"
    "../ext/libtess2/Include"
    "../ext"
    "../geometry"
    "../geometry/cgal"
    "../geometry/manifold"
    "../glview"
    "../glview/preview"
    "../glview/cgal"
    "../gui"
    "../io"
    "../platform"
    "../utils"
    "${EIGEN3_INCLUDE_DIR}"
    "${GLIB2_INCLUDE_DIRS}"
    "${FREETYPE_INCLUDE_DIRS}"
)

target_compile_features(openscad PUBLIC cxx_std_17)
