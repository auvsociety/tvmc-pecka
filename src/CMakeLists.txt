
# to interpolate thrust values
find_package(libInterpolate REQUIRED)

add_library(
    thruster_config
    thruster-config/thruster_config.cpp
    thruster-config/json.hpp
    thruster-config/csv.h
)

target_link_libraries(thruster_config ${catkin_LIBRARIES})

add_executable(pwmc
    reporters/pwm.cpp
)

target_link_libraries(pwmc ${catkin_LIBRARIES} thruster_config libInterpolate::Interpolate)

add_executable(tvmc 
    motion-controller/controller.cpp
    reporters/thrust.cpp
    PID-Controller/PID_controller.cpp
)

target_link_libraries(tvmc ${catkin_LIBRARIES} thruster_config)
