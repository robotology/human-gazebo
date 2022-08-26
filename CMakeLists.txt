cmake_minimum_required(VERSION 3.5)

project(human-gazebo
        VERSION 1.1.0)

set (HUMAN_GAZEBO_FILES humanSubject01/humanSubject01_66dof.urdf
                        humanSubject01/humanSubject01_48dof.urdf
                        humanSubject02/humanSubject02_66dof.urdf
			humanSubject02/humanSubject02_48dof.urdf
			humanSubject03/humanSubject03_66dof.urdf
			humanSubject03/humanSubject03_48dof.urdf
                        humanSubject04/humanSubject04_66dof.urdf
			humanSubject04/humanSubject04_48dof.urdf
			humanSubject05/humanSubject05_66dof.urdf
			humanSubject05/humanSubject05_48dof.urdf
			humanSubject06/humanSubject06_66dof.urdf
			humanSubject06/humanSubject06_48dof.urdf
			humanSubject07/humanSubject07_66dof.urdf
			humanSubject07/humanSubject07_48dof.urdf
			humanSubject08/humanSubject08_66dof.urdf
			humanSubject08/humanSubject08_48dof.urdf)

# Install human urdf models in human-gazebo directory
install(FILES ${HUMAN_GAZEBO_FILES} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME})

# Install human urdf models in HDERviz directory for ros
install(FILES ${HUMAN_GAZEBO_FILES} DESTINATION ${CMAKE_INSTALL_PREFIX}/share/HDERviz/urdfs)
