# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "hw_3: 0 messages, 1 services")

set(MSG_I_FLAGS "-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(hw_3_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/nik/ROS_ws/src/hw_3/srv/CustomTurtleMovement.srv" NAME_WE)
add_custom_target(_hw_3_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "hw_3" "/home/nik/ROS_ws/src/hw_3/srv/CustomTurtleMovement.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages

### Generating Services
_generate_srv_cpp(hw_3
  "/home/nik/ROS_ws/src/hw_3/srv/CustomTurtleMovement.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hw_3
)

### Generating Module File
_generate_module_cpp(hw_3
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hw_3
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(hw_3_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(hw_3_generate_messages hw_3_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nik/ROS_ws/src/hw_3/srv/CustomTurtleMovement.srv" NAME_WE)
add_dependencies(hw_3_generate_messages_cpp _hw_3_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hw_3_gencpp)
add_dependencies(hw_3_gencpp hw_3_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hw_3_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages

### Generating Services
_generate_srv_eus(hw_3
  "/home/nik/ROS_ws/src/hw_3/srv/CustomTurtleMovement.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hw_3
)

### Generating Module File
_generate_module_eus(hw_3
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hw_3
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(hw_3_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(hw_3_generate_messages hw_3_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nik/ROS_ws/src/hw_3/srv/CustomTurtleMovement.srv" NAME_WE)
add_dependencies(hw_3_generate_messages_eus _hw_3_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hw_3_geneus)
add_dependencies(hw_3_geneus hw_3_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hw_3_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages

### Generating Services
_generate_srv_lisp(hw_3
  "/home/nik/ROS_ws/src/hw_3/srv/CustomTurtleMovement.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hw_3
)

### Generating Module File
_generate_module_lisp(hw_3
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hw_3
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(hw_3_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(hw_3_generate_messages hw_3_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nik/ROS_ws/src/hw_3/srv/CustomTurtleMovement.srv" NAME_WE)
add_dependencies(hw_3_generate_messages_lisp _hw_3_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hw_3_genlisp)
add_dependencies(hw_3_genlisp hw_3_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hw_3_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages

### Generating Services
_generate_srv_nodejs(hw_3
  "/home/nik/ROS_ws/src/hw_3/srv/CustomTurtleMovement.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hw_3
)

### Generating Module File
_generate_module_nodejs(hw_3
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hw_3
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(hw_3_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(hw_3_generate_messages hw_3_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nik/ROS_ws/src/hw_3/srv/CustomTurtleMovement.srv" NAME_WE)
add_dependencies(hw_3_generate_messages_nodejs _hw_3_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hw_3_gennodejs)
add_dependencies(hw_3_gennodejs hw_3_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hw_3_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages

### Generating Services
_generate_srv_py(hw_3
  "/home/nik/ROS_ws/src/hw_3/srv/CustomTurtleMovement.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hw_3
)

### Generating Module File
_generate_module_py(hw_3
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hw_3
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(hw_3_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(hw_3_generate_messages hw_3_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/nik/ROS_ws/src/hw_3/srv/CustomTurtleMovement.srv" NAME_WE)
add_dependencies(hw_3_generate_messages_py _hw_3_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(hw_3_genpy)
add_dependencies(hw_3_genpy hw_3_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS hw_3_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hw_3)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/hw_3
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(hw_3_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hw_3)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/hw_3
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(hw_3_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hw_3)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/hw_3
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(hw_3_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hw_3)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/hw_3
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(hw_3_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hw_3)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hw_3\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/hw_3
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(hw_3_generate_messages_py std_msgs_generate_messages_py)
endif()
