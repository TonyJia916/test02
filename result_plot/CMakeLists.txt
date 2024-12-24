aux_source_directory(. DIR_LIB_SRCS)
set(CMAKE_CXX_STANDARD 17)

# Library target
add_library(ResultPlot INTERFACE)

target_compile_features(ResultPlot INTERFACE cxx_std_17)

# TODO: Use `Development.Embed` component when requiring cmake >= 3.18
find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
include_directories(/home/tan20/.local/lib/python3.8/site-packages/numpy/core/include)
target_include_directories(ResultPlot PRIVATE ${PYTHON_INCLUDE_DIRS})
if (Python3_FOUND)
message(STATUS "tests: ${Python3_EXECUTABLE}")
endif()

target_link_libraries(ResultPlot INTERFACE
  Python3::Python
  Python3::Module
)
find_package(Python3 COMPONENTS NumPy)
target_link_libraries(ResultPlot INTERFACE Python3::NumPy)

 

