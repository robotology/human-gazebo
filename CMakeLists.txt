cmake_minimum_required(VERSION 3.5)

project(human-gazebo)

set (HUMAN_GAZEBO_FILES humanSubject01/humanSubject01_66dof.urdf
                        humanSubject02/humanSubject02_66dof.urdf
			humanSubject03/humanSubject03_66dof.urdf
                        humanSubject04/humanSubject04_66dof.urdf
			humanSubject05/humanSubject05_66dof.urdf
			humanSubject06/humanSubject06_66dof.urdf
			humanSubject07/humanSubject07_66dof.urdf)

# Install human urdf models in human-gazebo directory
install(FILES ${HUMAN_GAZEBO_FILES} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME})

# Install human urdf models in HDERviz directory for ros
install(FILES ${HUMAN_GAZEBO_FILES} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/HDERviz/urdfs)
