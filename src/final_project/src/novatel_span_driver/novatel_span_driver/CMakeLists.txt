cmake_minimum_required(VERSION 2.8.3)
project(novatel_span_driver)

find_package(catkin REQUIRED COMPONENTS roslaunch roslint rostest)

catkin_python_setup()

catkin_package()


install(PROGRAMS scripts/driver
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

roslint_python()

if(CATKIN_ENABLE_TESTING)
  add_rostest(test/propak6-basic-fix.test)
  add_rostest(test/propak6-ins-ppp.test)

  roslaunch_add_file_check(launch/example.launch)
endif()
