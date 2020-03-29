project(mqtt_rotary_encoder)

cmake_minimum_required(VERSION 3.13)

set (CMAKE_CXX_STANDARD 14)
set( CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR} /usr/local/lib/cmake/PahoMqttCpp)
FIND_PACKAGE( Boost 1.67 COMPONENTS program_options thread REQUIRED )
FIND_PACKAGE( PahoMqttCpp REQUIRED )
FIND_PACKAGE( pigpio REQUIRED )
INCLUDE_DIRECTORIES( ${Boost_INCLUDE_DIR} )

ADD_EXECUTABLE( test_rotary_encoder test_rotary_encoder.cpp rotary_encoder.cpp)
ADD_EXECUTABLE( mqtt_pigpio mqtt_pigpio.cpp rotary_encoder.cpp)

TARGET_LINK_LIBRARIES( test_rotary_encoder ${pigpio_LIBRARY} Boost::chrono Boost::thread PahoMqttCpp::paho-mqttpp3 )
TARGET_LINK_LIBRARIES( mqtt_pigpio ${pigpio_LIBRARY} Boost::program_options Boost::chrono Boost::thread PahoMqttCpp::paho-mqttpp3 )
