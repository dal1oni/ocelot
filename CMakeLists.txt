cmake_minimum_required(VERSION 3.11)

project(ocelot)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(cmake/djinni_process_idl.cmake)

SET(DJINNI_CMD ${CMAKE_CURRENT_SOURCE_DIR}/djinni)
set(DJINNI_EXECUTABLE ${CMAKE_CURRENT_SOURCE_DIR}/djinni)
set(PYTHON_EXECUTABLE python3)

add_subdirectory(djinni-support-lib)

djinni_process_idl(
  DJINNI_IDL_FILE idl/ocelot.djinni
  GENERATED_OUT_DIR generated
  VARIABLE_PREFIX DJINNI_OCELOT
  NAMESPACE ocelot::djinni
  PYTHON_PACKAGE ocelot
)

include(${CMAKE_CURRENT_BINARY_DIR}/ocelot.djinni.cmake)

message("DJINNI_OCELOT_CPP_SOURCE = ${DJINNI_OCELOT_CPP_SOURCE}")
message("DJINNI_OCELOT_CPP_HEADER = ${DJINNI_OCELOT_CPP_HEADER}")
message("DJINNI_OCELOT_CWRAPPER_SOURCE = ${DJINNI_OCELOT_CWRAPPER_SOURCE}")

set(DJINNI_OCELOT_CWRAPPER_HEADER_FULL ${DJINNI_OCELOT_CWRAPPER_HEADER})
list(TRANSFORM DJINNI_OCELOT_CWRAPPER_HEADER_FULL PREPEND "${CMAKE_SOURCE_DIR}/")

message("DJINNI_OCELOT_CWRAPPER_HEADER_FULL = ${DJINNI_OCELOT_CWRAPPER_HEADER_FULL}")
  add_custom_target(PyCFFIlib_cffi ALL
    ${CMAKE_COMMAND} -E env PYTHONPATH=${PYTHONPATH} LIBRARY_PATH=$<TARGET_FILE_DIR:ocelot> ${PYTHON_EXECUTABLE}
        ${CMAKE_SOURCE_DIR}/${DJINNI_OCELOT_PYCFFI_SOURCE}
        ${CMAKE_SOURCE_DIR}/djinni-support-lib/djinni/cwrapper/wrapper_marshal.h
        ${DJINNI_OCELOT_CWRAPPER_HEADER_FULL}
#        ${CMAKE_CURRENT_SOURCE_DIR}/djinni-support-lib/handwritten-src/cwrapper/limits_helper.h
    DEPENDS ocelot ${DJINNI_OCELOT_PYCFFI_SOURCE} ${DJINNI_OCELOT_PY_SOURCE} ${DJINNI_OCELOT_CWRAPPER_HEADER}
    COMMENT "Building CFFI lib"
    VERBATIM
    WORKING_DIRECTORY $<TARGET_FILE_DIR:ocelot>
  )

add_library(ocelot
    SHARED
    # all generated headers
    ${DJINNI_OCELOT_CPP_SOURCE}
    # until source groups added
    ${DJINNI_OCELOT_CPP_HEADER}
    ${DJINNI_OCELOT_CWRAPPER_SOURCE}
    ${DJINNI_OCELOT_CWRAPPER_HEADER}
    ${DJINNI_OCELOT_PYCFFI_SOURCE}
    # our implementation
    src/wave.cpp
)

set_property (TARGET ocelot PROPERTY CXX_STANDARD 20)

target_include_directories(ocelot PUBLIC generated/include)
target_include_directories(ocelot PUBLIC generated/include/ocelot/djinni/cpp)
target_include_directories(ocelot PUBLIC generated/include/ocelot/djinni/c-wrapper)
target_include_directories(ocelot PUBLIC eigen)

target_link_libraries(ocelot PUBLIC djinni-support-lib::djinni-support-lib)
