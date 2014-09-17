#
# Dynamics3d Gripper plugin
#

#
# Plugin headers
#
set(ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHYSICS_ENGINES_DYNAMICS3D_PLUGINS_GRIPPER
    dynamics3d_gripper_plugin.h)

#
# Plugin sources
#
set(ARGOS3_SOURCES_PLUGINS_SIMULATOR_PHYSICS_ENGINES_DYNAMICS3D_PLUGINS_GRIPPER
    ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHYSICS_ENGINES_DYNAMICS3D_PLUGINS_GRIPPER}
    dynamics3d_gripper_plugin.cpp)

#
# Create the gripper plugin
#
include_directories(
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletCollision/BroadphaseCollision
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletCollision/CollisionDispatch
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletCollision/CollisionShapes
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletCollision/Gimpact
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletCollision/NarrowPhaseCollision
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletDynamics/ConstraintSolver
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletDynamics/Dynamics
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletDynamics/Vehicle
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/BulletDynamics/Character
   ${CMAKE_SOURCE_DIR}/plugins/simulator/physics_engines/dynamics3d/bullet/LinearMath)
add_library(argos3plugin_${ARGOS_BUILD_FOR}_dyn3dplugin_gripper SHARED ${ARGOS3_SOURCES_PLUGINS_SIMULATOR_PHYSICS_ENGINES_DYNAMICS3D_PLUGINS_GRIPPER})
target_link_libraries(argos3plugin_${ARGOS_BUILD_FOR}_dyn3dplugin_gripper
                      argos3plugin_${ARGOS_BUILD_FOR}_dynamics3d)

install(FILES ${ARGOS3_HEADERS_PLUGINS_SIMULATOR_PHYSICS_ENGINES_DYNAMICS3D_PLUGINS_GRIPPER} DESTINATION include/argos3/plugins/physics_engines/dynamics3d/plugins/gripper)

install(TARGETS argos3plugin_${ARGOS_BUILD_FOR}_dyn3dplugin_gripper
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib/argos3
  ARCHIVE DESTINATION lib/argos3)
