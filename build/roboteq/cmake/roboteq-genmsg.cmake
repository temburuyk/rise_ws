# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "roboteq: 1 messages, 0 services")

set(MSG_I_FLAGS "-Iroboteq:/home/yashwant/rise_ws/src/roboteq/msg;-Igeometry_msgs:/home/yashwant/y_ws/src/common_msgs-jade-devel/geometry_msgs/msg;-Isensor_msgs:/home/yashwant/y_ws/src/common_msgs-jade-devel/sensor_msgs/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(roboteq_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/yashwant/rise_ws/src/roboteq/msg/roboteq_msg.msg" NAME_WE)
add_custom_target(_roboteq_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "roboteq" "/home/yashwant/rise_ws/src/roboteq/msg/roboteq_msg.msg" "std_msgs/Header"
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(roboteq
  "/home/yashwant/rise_ws/src/roboteq/msg/roboteq_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roboteq
)

### Generating Services

### Generating Module File
_generate_module_cpp(roboteq
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roboteq
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(roboteq_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(roboteq_generate_messages roboteq_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yashwant/rise_ws/src/roboteq/msg/roboteq_msg.msg" NAME_WE)
add_dependencies(roboteq_generate_messages_cpp _roboteq_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(roboteq_gencpp)
add_dependencies(roboteq_gencpp roboteq_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roboteq_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(roboteq
  "/home/yashwant/rise_ws/src/roboteq/msg/roboteq_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roboteq
)

### Generating Services

### Generating Module File
_generate_module_lisp(roboteq
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roboteq
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(roboteq_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(roboteq_generate_messages roboteq_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yashwant/rise_ws/src/roboteq/msg/roboteq_msg.msg" NAME_WE)
add_dependencies(roboteq_generate_messages_lisp _roboteq_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(roboteq_genlisp)
add_dependencies(roboteq_genlisp roboteq_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roboteq_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(roboteq
  "/home/yashwant/rise_ws/src/roboteq/msg/roboteq_msg.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roboteq
)

### Generating Services

### Generating Module File
_generate_module_py(roboteq
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roboteq
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(roboteq_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(roboteq_generate_messages roboteq_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/yashwant/rise_ws/src/roboteq/msg/roboteq_msg.msg" NAME_WE)
add_dependencies(roboteq_generate_messages_py _roboteq_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(roboteq_genpy)
add_dependencies(roboteq_genpy roboteq_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS roboteq_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roboteq)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/roboteq
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_cpp)
  add_dependencies(roboteq_generate_messages_cpp geometry_msgs_generate_messages_cpp)
endif()
if(TARGET sensor_msgs_generate_messages_cpp)
  add_dependencies(roboteq_generate_messages_cpp sensor_msgs_generate_messages_cpp)
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(roboteq_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roboteq)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/roboteq
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_lisp)
  add_dependencies(roboteq_generate_messages_lisp geometry_msgs_generate_messages_lisp)
endif()
if(TARGET sensor_msgs_generate_messages_lisp)
  add_dependencies(roboteq_generate_messages_lisp sensor_msgs_generate_messages_lisp)
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(roboteq_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roboteq)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roboteq\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/roboteq
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET geometry_msgs_generate_messages_py)
  add_dependencies(roboteq_generate_messages_py geometry_msgs_generate_messages_py)
endif()
if(TARGET sensor_msgs_generate_messages_py)
  add_dependencies(roboteq_generate_messages_py sensor_msgs_generate_messages_py)
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(roboteq_generate_messages_py std_msgs_generate_messages_py)
endif()
