## ![http://bulletphysics.org/wordpress/wp-content/themes/atahualpa333/images/bullet_logo-210-86.png](http://bulletphysics.org/wordpress/wp-content/themes/atahualpa333/images/bullet_logo-210-86.png) ![http://jbullet-jme.googlecode.com/svn/wiki/images/bouncejme.png](http://jbullet-jme.googlecode.com/svn/wiki/images/bouncejme.png) ##
The aim of jbullet-jme is to simplify the use of jbullet and adapt it to jme2. jbullet is a physics library completely written in Java.

jbullet-jme wraps jbullet into its own set of objects which integrate into the scenegraph of jme2 and are updated by the physics engine to move according to the physics.

Other than jmephysics2, which aims at integrating different physics engines in one wrapper library, jbullet-jme is pretty much a direct wrapper of jbullet. This allows using some of the special functions of bullet (like vehicles) and adds the possibility of developing a native bullet implementation of jbullet-jme in the future, supporting GPU use etc. (see Native\_Version)


Since this wrapper is based on bullet physics and its java port jbullet, you might find help for some problems on the bullet physics homepage: http://www.bulletphysics.org

## Implemented Features ##
  * [PhysicsNodes](PhysicsNode.md) (RigidBodys)
  * Forces
  * CollisionShapes
    * Box
    * Sphere
    * Capsule
    * Cylinder
    * Compound
    * Mesh
  * [Joints](PhysicsJoint.md)
    * HingeJoint (incl. motors)
    * Point2PointJoint
    * SliderJoint (incl. motors)
    * ConeJoint
    * 6DofJoint (incl motors)
  * CollisionEvents
  * [Vehicles](PhysicsVehicleNode.md)
  * [Character Nodes](PhysicsCharacterNode.md)
  * [GameTaskQueue](Multithreading.md) for optional multithreading

## This wiki ##
If you feel like some information is missing and you can put it together, please add a comment to the wiki page you'd like to extend containing the relevant info and it will be added to the wiki.

## JavaDoc ##
http://jbullet-jme.googlecode.com/svn/trunk/jbullet-jme/javadoc/index.html