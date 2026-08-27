cmake_minimum_required(VERSION 3.8)
project(dracoviloc_audio_fusion)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/odas_ekf_adapter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME odas_ekf_adapter
)

install(PROGRAMS
  scripts/ekf_fusion_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ekf_fusion_node
)

install(PROGRAMS
  scripts/ast_confidence_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME ast_confidence_monitor
)

install(PROGRAMS
  scripts/yolo_ekf_adapter.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME yolo_ekf_adapter
)

# ast_classifier_node.py and gre_classifier_node.py are not installed as
# ros2-run entry points: each must run under its own venv's interpreter
# (TensorRT/pycuda/transformers for AST, TensorRT/pycuda/soundfile/pandas for
# GRE - neither lives in the system ROS Python), so the launch file invokes
# each by full path with ExecuteProcess instead. Their source directories are
# installed as a share/ tree so those paths are stable and independent of
# where this package was built from.
install(DIRECTORY ast DESTINATION share/${PROJECT_NAME})
install(DIRECTORY gre DESTINATION share/${PROJECT_NAME})
install(DIRECTORY models DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
