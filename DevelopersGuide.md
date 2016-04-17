## Introduction ##
This page is intended for people who want to improve / extend jbullet-jme. To get access, send your googlecode account email address to normen667.

## JAVA version ##
jbullet-jme is a pretty straightforward jbullet wrapper that mostly encapsules jbullet objects in similar own objects and makes the methods of the jbullet objects accessible, mostly by mirroring them. So to understand jbullet-jme it is advised to look at jbullet / bullet.

  * PhyiscsNodes contain a RigidBody
  * PhysicsGhostNodes contain a PairCachingGhostObject

Some objects integrate functions of RigidBodys with Constraints, such as PhysicsVehicleNode or PhysicsCharacterNode.

  * PhysicsVehicleNodes are PhysicsNodes with a VehicleRaycaster and a RayCastVehicle
  * PhysicsCharacterNodes are PhysicsGhostNodes  with a KinematicCharacterController

## Native version ##
The beginnings of a native version are in the branches/bullet-jme-work svn folder.

#### BulletJmeJavaMethods.cpp ####
This file contains the native methods of **all** bullet-jme java classes. Mostly it routes the calls to the created BulletJmePhysicsSpace object.

#### BulletJmePhysicsSpace.cpp ####
In this object the bullet DynamicsWorld and objects are stored and almost all interaction with them happens via the methods of this class.

Since java does not allow storing pointers to C++ objects, all bullet objects are stored in maps, with a "jlong" as pointer to them. This long value is also stored in the corrsponding java object. When a java object wants to call some command on the native object, it also sends the long identifier of that C++ object.

#### BulletJmeMotionState.cpp ####
This is the MotionState attached to RigidBodys which receives the location/rotation changes of the RigidBody and synchronizes the corresponding JME Node. It stores a java pointer to the corresponding PhysicsNode.