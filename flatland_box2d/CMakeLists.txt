
set(BOX2D_Collision_SRCS
	src/Collision/b2BroadPhase.cpp
	src/Collision/b2CollideCircle.cpp
	src/Collision/b2CollideEdge.cpp
	src/Collision/b2CollidePolygon.cpp
	src/Collision/b2Collision.cpp
	src/Collision/b2Distance.cpp
	src/Collision/b2DynamicTree.cpp
	src/Collision/b2TimeOfImpact.cpp
)
set(BOX2D_Collision_HDRS
	
	include/Box2D/Collision/b2Collision.h
	include/Box2D/Collision/b2Distance.h
	include/Box2D/Collision/b2DynamicTree.h
	include/Box2D/Collision/b2TimeOfImpact.h
)
set(BOX2D_Shapes_SRCS
	src/Collision/Shapes/b2CircleShape.cpp
	src/Collision/Shapes/b2EdgeShape.cpp
	src/Collision/Shapes/b2ChainShape.cpp
	src/Collision/Shapes/b2PolygonShape.cpp
)
set(BOX2D_Shapes_HDRS
	include/Box2D/Collision/Shapes/b2CircleShape.h
	include/Box2D/Collision/Shapes/b2EdgeShape.h
	include/Box2D/Collision/Shapes/b2ChainShape.h
	include/Box2D/Collision/Shapes/b2PolygonShape.h
	include/Box2D/Collision/Shapes/b2Shape.h
)
set(BOX2D_Common_SRCS
	src/Common/b2BlockAllocator.cpp
	src/Common/b2Draw.cpp
	src/Common/b2Math.cpp
	src/Common/b2Settings.cpp
	src/Common/b2StackAllocator.cpp
	src/Common/b2Timer.cpp
)
set(BOX2D_Common_HDRS
	include/Box2D/Common/b2BlockAllocator.h
	include/Box2D/Common/b2Draw.h
	include/Box2D/Common/b2GrowableStack.h
	include/Box2D/Common/b2Math.h
	include/Box2D/Common/b2Settings.h
	include/Box2D/Common/b2StackAllocator.h
	include/Box2D/Common/b2Timer.h
)
set(BOX2D_Dynamics_SRCS
	src/Dynamics/b2Body.cpp
	src/Dynamics/b2ContactManager.cpp
	src/Dynamics/b2Fixture.cpp
	src/Dynamics/b2Island.cpp
	src/Dynamics/b2World.cpp
	src/Dynamics/b2WorldCallbacks.cpp
)
set(BOX2D_Dynamics_HDRS
	include/Box2D/Dynamics/b2Body.h
	include/Box2D/Dynamics/b2ContactManager.h
	include/Box2D/Dynamics/b2Fixture.h
	include/Box2D/Dynamics/b2Island.h
	include/Box2D/Dynamics/b2TimeStep.h
	include/Box2D/Dynamics/b2World.h
	include/Box2D/Dynamics/b2WorldCallbacks.h
)
set(BOX2D_Contacts_SRCS
	src/Dynamics/Contacts/b2CircleContact.cpp
	src/Dynamics/Contacts/b2Contact.cpp
	src/Dynamics/Contacts/b2ContactSolver.cpp
	src/Dynamics/Contacts/b2PolygonAndCircleContact.cpp
	src/Dynamics/Contacts/b2EdgeAndCircleContact.cpp
	src/Dynamics/Contacts/b2EdgeAndPolygonContact.cpp
	src/Dynamics/Contacts/b2ChainAndCircleContact.cpp
	src/Dynamics/Contacts/b2ChainAndPolygonContact.cpp
	src/Dynamics/Contacts/b2PolygonContact.cpp
)
set(BOX2D_Contacts_HDRS
	include/Box2D/Dynamics/Contacts/b2CircleContact.h
	include/Box2D/Dynamics/Contacts/b2Contact.h
	include/Box2D/Dynamics/Contacts/b2ContactSolver.h
	include/Box2D/Dynamics/Contacts/b2PolygonAndCircleContact.h
	include/Box2D/Dynamics/Contacts/b2EdgeAndCircleContact.h
	include/Box2D/Dynamics/Contacts/b2EdgeAndPolygonContact.h
	include/Box2D/Dynamics/Contacts/b2ChainAndCircleContact.h
	include/Box2D/Dynamics/Contacts/b2ChainAndPolygonContact.h
	include/Box2D/Dynamics/Contacts/b2PolygonContact.h
)
set(BOX2D_Joints_SRCS
	src/Dynamics/Joints/b2DistanceJoint.cpp
	src/Dynamics/Joints/b2FrictionJoint.cpp
	src/Dynamics/Joints/b2GearJoint.cpp
	src/Dynamics/Joints/b2Joint.cpp
	src/Dynamics/Joints/b2MotorJoint.cpp
	src/Dynamics/Joints/b2MouseJoint.cpp
	src/Dynamics/Joints/b2PrismaticJoint.cpp
	src/Dynamics/Joints/b2PulleyJoint.cpp
	src/Dynamics/Joints/b2RevoluteJoint.cpp
	src/Dynamics/Joints/b2RopeJoint.cpp
	src/Dynamics/Joints/b2WeldJoint.cpp
	src/Dynamics/Joints/b2WheelJoint.cpp
)
set(BOX2D_Joints_HDRS
	include/Box2D/Dynamics/Joints/b2DistanceJoint.h
	include/Box2D/Dynamics/Joints/b2FrictionJoint.h
	include/Box2D/Dynamics/Joints/b2GearJoint.h
	include/Box2D/Dynamics/Joints/b2Joint.h
	include/Box2D/Dynamics/Joints/b2MotorJoint.h
	include/Box2D/Dynamics/Joints/b2MouseJoint.h
	include/Box2D/Dynamics/Joints/b2PrismaticJoint.h
	include/Box2D/Dynamics/Joints/b2PulleyJoint.h
	include/Box2D/Dynamics/Joints/b2RevoluteJoint.h
	include/Box2D/Dynamics/Joints/b2RopeJoint.h
	include/Box2D/Dynamics/Joints/b2WeldJoint.h
	include/Box2D/Dynamics/Joints/b2WheelJoint.h
)
set(BOX2D_Rope_SRCS
	src/Rope/b2Rope.cpp
)
set(BOX2D_Rope_HDRS
	include/Box2D/Rope/b2Rope.h
)
set(BOX2D_General_HDRS
	include/Box2D/Box2D.h
)

cmake_minimum_required(VERSION 3.5)
project(flatland_box2d)

find_package(ament_cmake REQUIRED)

include_directories(include)

add_library(flatland_box2d_lib SHARED
	${BOX2D_General_HDRS}
	${BOX2D_Joints_SRCS}
	${BOX2D_Joints_HDRS}
	${BOX2D_Contacts_SRCS}
	${BOX2D_Contacts_HDRS}
	${BOX2D_Dynamics_SRCS}
	${BOX2D_Dynamics_HDRS}
	${BOX2D_Common_SRCS}
	${BOX2D_Common_HDRS}
	${BOX2D_Shapes_SRCS}
	${BOX2D_Shapes_HDRS}
	${BOX2D_Collision_SRCS}
	${BOX2D_Collision_HDRS}
	${BOX2D_Rope_SRCS}
	${BOX2D_Rope_HDRS}
)
set_target_properties(flatland_box2d_lib PROPERTIES
	GIT_COMMIT_HASH f655c603ba9d83f07fc566d38d2654ba35739102
)

ament_export_targets(export_flatland_box2d HAS_LIBRARY_TARGET)
ament_export_libraries(flatland_box2d_lib)
ament_export_include_directories(include)

ament_package()

install( 
  TARGETS flatland_box2d_lib
  EXPORT export_flatland_box2d
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# Mark cpp header files for installation
install(
  DIRECTORY include/
  DESTINATION include
  FILES_MATCHING PATTERN "*.h"
)