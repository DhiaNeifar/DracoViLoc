cmake_minimum_required(VERSION 3.8)
project(dracoviloc_odas)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/uma16_feeder_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME uma16_feeder
)

install(PROGRAMS
  scripts/audio_target_tracker.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME audio_target_tracker
)

install(PROGRAMS
  scripts/analyze_audio_band.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME analyze_audio_band
)

install(PROGRAMS
  scripts/generate_beacon_noise.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME generate_beacon_noise
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()
