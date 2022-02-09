include_guard(GLOBAL)
set(djinni_cmake_utils_dir ${CMAKE_CURRENT_LIST_DIR} CACHE INTERNAL "")

macro(djinni_process_idl)

  set(options )
  set(values DJINNI_IDL_FILE GENERATED_OUT_DIR VARIABLE_PREFIX PYTHON_PACKAGE NAMESPACE)
  set(lists )

  cmake_parse_arguments(_idldef
      "${options}" "${values}" "${lists}" ${ARGN})

  set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
      ${_idldef_DJINNI_IDL_FILE}
  )

  find_program(DJINNI_CMD djinni DOC "Path to djinni executable" REQUIRED)


  get_filename_component(name_wle ${_idldef_DJINNI_IDL_FILE} NAME_WLE)
  get_filename_component(name ${_idldef_DJINNI_IDL_FILE} NAME)
  get_filename_component(name_real ${_idldef_DJINNI_IDL_FILE} REALPATH)

  set(_idldef_LIST_OUT_FILE ${CMAKE_CURRENT_BINARY_DIR}/cmake_${name}.generated.txt)
  set(_idldef_CMAKE_FILE ${CMAKE_CURRENT_BINARY_DIR}/${name}.cmake)

  set(skip_djinni_call OFF)
  if(EXISTS ${_idldef_LIST_OUT_FILE})
      if(NOT ${name_real} IS_NEWER_THAN ${_idldef_LIST_OUT_FILE})
          message("-- djiini call not required, already processed ${name}.")
          set(skip_djinni_call ON)
      endif()
  endif()

  if(NOT skip_djinni_call)
    string(REPLACE "::" "/" namespace_path ${_idldef_NAMESPACE})

    set(LIST_FILE ${_idldef_LIST_OUT_FILE})
    set(OUT_FILE ${_idldef_CMAKE_FILE})
    set(VARIABLE_PREFIX ${_idldef_VARIABLE_PREFIX})

    set(arg_py_out "../${_idldef_GENERATED_OUT_DIR}/${namespace_path}/py/")
    set(arg_pycffi_out "../${_idldef_GENERATED_OUT_DIR}/${namespace_path}/pycffi/")
    set(arg_cwrapper_out "../${_idldef_GENERATED_OUT_DIR}/${namespace_path}/c-wrapper/")
    set(arg_cwrapper_header_out "../${_idldef_GENERATED_OUT_DIR}/include/${namespace_path}/c-wrapper")

    set(arg_cpp_out "../${_idldef_GENERATED_OUT_DIR}/${namespace_path}/cpp/")
    set(arg_cpp_header_out "../${_idldef_GENERATED_OUT_DIR}/include/${namespace_path}/cpp")
    set(arg_yaml_out "../${_idldef_GENERATED_OUT_DIR}/${namespace_path}/yaml/${name}.yaml")

    set(djinni_args
        --idl "../${_idldef_DJINNI_IDL_FILE}"
        --py-out ${arg_py_out}
        --py-import-prefix "${_idldef_PYTHON_PACKAGE}.djinni."
        --pycffi-out ${arg_pycffi_out}
        --pycffi-package-name PyCFFIlib
        --pycffi-dynamic-lib-list ${_idldef_PYTHON_PACKAGE}
        --c-wrapper-out ${arg_cwrapper_out}
        --c-wrapper-header-out ${arg_cwrapper_header_out}
        --cpp-out ${arg_cpp_out}
        --cpp-header-out ${arg_cpp_header_out}
        --cpp-include-prefix ${namespace_path}/cpp/
        --cpp-namespace ${_idldef_NAMESPACE}
        --yaml-out ${arg_yaml_out}
        --list-out-files ${_idldef_LIST_OUT_FILE}
        --ident-cpp-file FooBar
    )

    message("Generating djinni source files for ${name}")
    message("${DJINNI_CMD} ${djinni_args}")
    execute_process(
      COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_CURRENT_LIST_DIR}/${_idldef_GENERATED_OUT_DIR}
      COMMAND ${DJINNI_CMD} ${djinni_args}
      WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}
     # COMMAND_ERROR_IS_FATAL LAST
      #COMMAND_ECHO STDOUT
    )

    message("Generating djinni cmake include file for ${name}")
    FILE(READ "${LIST_FILE}" generated_files)
    STRING(REGEX REPLACE ";" "\\\\;" generated_files "${generated_files}")
    STRING(REGEX REPLACE "\n" ";" generated_files "${generated_files}")

    set(CPP_SOURCE "")
    set(CPP_HEADER "")
    set(CWRAPPER_SOURCE "")
    set(CWRAPPER_HEADER "")
    set(PY_SOURCE "")
    set(PYCFFI_SOURCE "")

    foreach(item ${generated_files})
      if(item MATCHES "^${arg_cpp_out}")
        list(APPEND CPP_SOURCE ${item})
      elseif(item MATCHES "^${arg_cpp_header_out}")
        list(APPEND CPP_HEADER ${item})
      elseif(item MATCHES "^${arg_cwrapper_out}")
        list(APPEND CWRAPPER_SOURCE ${item})
      elseif(item MATCHES "^${arg_cwrapper_header_out}")
        list(APPEND CWRAPPER_HEADER ${item})
      elseif(item MATCHES "^${arg_cpp_header_out}")
        list(APPEND CPP_HEADER ${item})
      elseif(item MATCHES "^${arg_py_out}")
        list(APPEND PY_SOURCE ${item})
      elseif(item MATCHES "^${arg_pycffi_out}")
        list(APPEND PYCFFI_SOURCE ${item})
      elseif(item MATCHES "^${arg_yaml_out}")
        # ignore
      else()
        message(WARNING "Unhandled generated file: " ${item})
      endif()

    endforeach()

    string (REPLACE ";" " " CPP_SOURCE "${CPP_SOURCE}")
    string (REPLACE ";" " " CPP_HEADER "${CPP_HEADER}")
    string (REPLACE ";" " " CWRAPPER_SOURCE "${CWRAPPER_SOURCE}")
    string (REPLACE ";" " " CWRAPPER_HEADER "${CWRAPPER_HEADER}")
    string (REPLACE ";" " " PY_SOURCE "${PY_SOURCE}")
    string (REPLACE ";" " " PYCFFI_SOURCE "${PYCFFI_SOURCE}")

    string (REPLACE "../" "" CPP_SOURCE "${CPP_SOURCE}")
    string (REPLACE "../" "" CPP_HEADER "${CPP_HEADER}")
    string (REPLACE "../" "" CWRAPPER_SOURCE "${CWRAPPER_SOURCE}")
    string (REPLACE "../" "" CWRAPPER_HEADER "${CWRAPPER_HEADER}")
    string (REPLACE "../" "" PY_SOURCE "${PY_SOURCE}")
    string (REPLACE "../" "" PYCFFI_SOURCE "${PYCFFI_SOURCE}")

    configure_file(${djinni_cmake_utils_dir}/djinni_generated_files.cmake.in
                  ${OUT_FILE}
                  @ONLY
    )
  endif()
endmacro()
