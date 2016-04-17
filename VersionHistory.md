## Versions ##

#### 0.9.6 alpha ([r569](https://code.google.com/p/jbullet-jme/source/detail?r=569)) ####
  * added debug view
  * added kinematic PhysicsNodes
  * add blank constructor for PhysicsNode
  * add rotation for CompoundCollisionShape children
  * add getHingeAngle()
  * add CompoundCollisionShape to debug view
  * add methods to set fall speed etc. to PhysicsCharacterNode

#### 0.9.5 alpha ([r509](https://code.google.com/p/jbullet-jme/source/detail?r=509)) ####
  * added fix to avoid calling of setLocalInertia() for static meshes (caused assertion error)
  * fixed mesh conversion, GIMPACT shapes work now
  * added GIMPACT type to automatic CollisionShape generation
  * added PhysicsCharacterNode constructor with CollisionShape
  * removed applying of rotation for PhysicsCharacterNodes (is always zero anyway)
  * PhysicsCharacterNode.jump() works now
  * PhysicsCharacterNode gravity works now

#### 0.9.4 alpha ([r476](https://code.google.com/p/jbullet-jme/source/detail?r=476)) ####
  * fixed activation of PhysicsNodes when applying forces
  * added PhysicsNode.activate()
  * added PhysicsNode.setSleepingThresholds()
  * fixed node update order when creating CollisionShape
  * added non-centric forces
  * added non-continuous force applying
  * added WheelInfo restLength support
  * preliminary fix for CollisionShape generation from BoundingVolumes with center!=0
  * lots of javadoc additions
  * added GImpactShape (not working yet)

#### 0.9.3 alpha ([r374](https://code.google.com/p/jbullet-jme/source/detail?r=374)) ####
  * some fixes in scale applying
  * improved key management in vehicle and character tests
  * fixed bug in PhysicsSpace.setGravity()
  * added get/setLinearVelocity
  * added CapsuleCollisionShape around X and Z axis
  * added accuracy option to PhysicsSpace
  * added Physics(Ghost)Node blank constructor
  * fixed CollisionShape generation from BoundingVolumes with center!=0

#### 0.9.2 alpha ([r348](https://code.google.com/p/jbullet-jme/source/detail?r=348)) ####
  * added wheel contact info (location,skid,object) to WheelInfo
  * added basic CompoundCollisionShape
  * added applying of rotation from lookAt() and rotateUpTo()
  * fixed some joint constructors without rotation parameter
  * fixed setLocalScale()
  * use userObject in RigidBody to link PhysicsNode

#### 0.9.1 alpha ([r336](https://code.google.com/p/jbullet-jme/source/detail?r=336)) ####
  * made setWorldRotation/Translation protected
  * added warp() to PhysicsCharacterNode
  * fixed bug in PhysicsHingeJoint.setLimit(low, high)
  * fixed creation of CollisionShape from BoundingVolume
  * CollisionShape rebuilding when scaling PhysicsNode

#### 0.9 alpha ([r256](https://code.google.com/p/jbullet-jme/source/detail?r=256)) ####
  * first released alpha testing version

## Version Numbering ##
jbulet-jme version numbers advance in the following way:

  * 3rd order: bugfix releases
    * 1.0.0 -> 1.0.1
  * 2nd order:  new feature releases
    * 1.0.0 -> 1.1.0
  * 1st order:  releases changing api / bringing massive changes
    * 1.0.0 -> 2.0.0