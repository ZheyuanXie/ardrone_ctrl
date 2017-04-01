cmake_minimum_required(VERSION 2.8.3)
project(ardrone_ctrl)

find_package(catkin REQUIRED)
catkin_package()

install(PROGRAMS ctrl.py
	gui.py
        DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
