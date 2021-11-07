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

add_library(ocelot
    SHARED
    # all genearted headers
    ${DJINNI_OCELOT_CPP_SOURCE}
    # until source groups added
    ${DJINNI_OCELOT_CPP_HEADER}
    ${DJINNI_OCELOT_CWRAPPER_SOURCE}
    ${DJINNI_OCELOT_CWRAPPER_HEADER}
    ${DJINNI_OCELOT_PYCFFI_SOURCE}
    # our implementation
    src/wave.cpp
)

target_include_directories(ocelot PUBLIC generated/include)
target_include_directories(ocelot PUBLIC generated/include/ocelot/djinni/cpp)
target_include_directories(ocelot PUBLIC generated/include/ocelot/djinni/c-wrapper)

target_link_libraries(ocelot PUBLIC djinni-support-lib::djinni-support-lib)
