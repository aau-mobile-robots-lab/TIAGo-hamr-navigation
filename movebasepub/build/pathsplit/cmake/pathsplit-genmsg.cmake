# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "pathsplit: 1 messages, 1 services")

set(MSG_I_FLAGS "-Ipathsplit:/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/msg;-Istd_msgs:/opt/ros/kinetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(pathsplit_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/srv/AddTwoInts.srv" NAME_WE)
add_custom_target(_pathsplit_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pathsplit" "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/srv/AddTwoInts.srv" ""
)

get_filename_component(_filename "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/msg/Num.msg" NAME_WE)
add_custom_target(_pathsplit_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "pathsplit" "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/msg/Num.msg" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(pathsplit
  "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pathsplit
)

### Generating Services
_generate_srv_cpp(pathsplit
  "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pathsplit
)

### Generating Module File
_generate_module_cpp(pathsplit
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pathsplit
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(pathsplit_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(pathsplit_generate_messages pathsplit_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(pathsplit_generate_messages_cpp _pathsplit_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/msg/Num.msg" NAME_WE)
add_dependencies(pathsplit_generate_messages_cpp _pathsplit_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pathsplit_gencpp)
add_dependencies(pathsplit_gencpp pathsplit_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pathsplit_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(pathsplit
  "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pathsplit
)

### Generating Services
_generate_srv_eus(pathsplit
  "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pathsplit
)

### Generating Module File
_generate_module_eus(pathsplit
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pathsplit
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(pathsplit_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(pathsplit_generate_messages pathsplit_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(pathsplit_generate_messages_eus _pathsplit_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/msg/Num.msg" NAME_WE)
add_dependencies(pathsplit_generate_messages_eus _pathsplit_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pathsplit_geneus)
add_dependencies(pathsplit_geneus pathsplit_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pathsplit_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(pathsplit
  "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pathsplit
)

### Generating Services
_generate_srv_lisp(pathsplit
  "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pathsplit
)

### Generating Module File
_generate_module_lisp(pathsplit
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pathsplit
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(pathsplit_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(pathsplit_generate_messages pathsplit_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(pathsplit_generate_messages_lisp _pathsplit_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/msg/Num.msg" NAME_WE)
add_dependencies(pathsplit_generate_messages_lisp _pathsplit_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pathsplit_genlisp)
add_dependencies(pathsplit_genlisp pathsplit_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pathsplit_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(pathsplit
  "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pathsplit
)

### Generating Services
_generate_srv_nodejs(pathsplit
  "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pathsplit
)

### Generating Module File
_generate_module_nodejs(pathsplit
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pathsplit
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(pathsplit_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(pathsplit_generate_messages pathsplit_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(pathsplit_generate_messages_nodejs _pathsplit_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/msg/Num.msg" NAME_WE)
add_dependencies(pathsplit_generate_messages_nodejs _pathsplit_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pathsplit_gennodejs)
add_dependencies(pathsplit_gennodejs pathsplit_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pathsplit_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(pathsplit
  "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/msg/Num.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pathsplit
)

### Generating Services
_generate_srv_py(pathsplit
  "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/srv/AddTwoInts.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pathsplit
)

### Generating Module File
_generate_module_py(pathsplit
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pathsplit
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(pathsplit_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(pathsplit_generate_messages pathsplit_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/srv/AddTwoInts.srv" NAME_WE)
add_dependencies(pathsplit_generate_messages_py _pathsplit_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/martinpc/PycharmProjects/TIAGo-hamr-navigation/movebasepub/src/pathsplit/msg/Num.msg" NAME_WE)
add_dependencies(pathsplit_generate_messages_py _pathsplit_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(pathsplit_genpy)
add_dependencies(pathsplit_genpy pathsplit_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS pathsplit_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pathsplit)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/pathsplit
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(pathsplit_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pathsplit)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/pathsplit
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(pathsplit_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pathsplit)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/pathsplit
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(pathsplit_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pathsplit)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/pathsplit
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(pathsplit_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pathsplit)
  install(CODE "execute_process(COMMAND \"/usr/bin/python2\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pathsplit\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/pathsplit
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(pathsplit_generate_messages_py std_msgs_generate_messages_py)
endif()
