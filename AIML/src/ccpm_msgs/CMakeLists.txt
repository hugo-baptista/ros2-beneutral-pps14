cmake_minimum_required(VERSION 3.8)
project(ccpm_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies



find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)  # Add other dependencies as needed

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)



find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CanMessage.msg"
  "msg/CommandMessage.msg"
  "msg/GPSMessage.msg"
  "msg/RadioMessage.msg"
  "msg/ModemStatus.msg"
  "msg/BrakeStatus.msg"
  "msg/SteeringStatus.msg"
  "msg/PowertrainStatus.msg"
  "msg/DeviceStatus.msg"
  "msg/NetworkStatus.msg"
  "msg/XP0Powertrain.msg"
  "srv/CreatePeriodicCan.srv"
  "srv/GetIMEI.srv"
  "srv/GetModemStatus.srv"
  "srv/MqttSend.srv"
  "srv/Restart.srv"
  "srv/SendByteArray.srv"
  "srv/Start.srv"
  "srv/Stop.srv"
  "srv/StopPeriodicCan.srv"
  "srv/ConfigureWiFiAP.srv"
  "action/ProcessCmd.action"
  DEPENDENCIES
    std_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()