# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "homework2: 0 messages, 1 services")

set(MSG_I_FLAGS "")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(homework2_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/student/ros/src/ugv_course/homework2/srv/string_cat.srv" NAME_WE)
add_custom_target(_homework2_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "homework2" "/home/student/ros/src/ugv_course/homework2/srv/string_cat.srv" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(homework2
  "/home/student/ros/src/ugv_course/homework2/srv/string_cat.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/homework2
)

### Generating Module File
_generate_module_cpp(homework2
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/homework2
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(homework2_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(homework2_generate_messages homework2_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student/ros/src/ugv_course/homework2/srv/string_cat.srv" NAME_WE)
add_dependencies(homework2_generate_messages_cpp _homework2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(homework2_gencpp)
add_dependencies(homework2_gencpp homework2_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS homework2_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(homework2
  "/home/student/ros/src/ugv_course/homework2/srv/string_cat.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/homework2
)

### Generating Module File
_generate_module_lisp(homework2
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/homework2
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(homework2_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(homework2_generate_messages homework2_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student/ros/src/ugv_course/homework2/srv/string_cat.srv" NAME_WE)
add_dependencies(homework2_generate_messages_lisp _homework2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(homework2_genlisp)
add_dependencies(homework2_genlisp homework2_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS homework2_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(homework2
  "/home/student/ros/src/ugv_course/homework2/srv/string_cat.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/homework2
)

### Generating Module File
_generate_module_py(homework2
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/homework2
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(homework2_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(homework2_generate_messages homework2_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/student/ros/src/ugv_course/homework2/srv/string_cat.srv" NAME_WE)
add_dependencies(homework2_generate_messages_py _homework2_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(homework2_genpy)
add_dependencies(homework2_genpy homework2_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS homework2_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/homework2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/homework2
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/homework2)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/homework2
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/homework2)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/homework2\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/homework2
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
