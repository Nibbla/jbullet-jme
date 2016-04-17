PhysicsNodes are normal JME Nodes that are updated by the physics thread. So to have some physics object in your world, you first create the jme geometry you'd like to move. Then you create a PhysicsNode with that object as a child. The PhysicsNode now contains all information about the location, rotation etc. of your physics object. The created geometry moves with it in the jme scenegraph, creating... visible physics :)

## Creating PhysicsNodes ##
To have a physics object you create a PhysicsNode with a CollisionShape. To do that you can simply pass a spatial to the PhysicsNode constructor and define a CollisionShape type and the CollisionShape will be created automatically from the BoundingVolume of the Spatial. You can also create your own collision shapes before, see [CollisionShapes](CollisionShapes.md)
```
//Create a jme sphere
Sphere sphere=new Sphere("physicssphere",16,16,1f);
//Create physics sphere from it
PhysicsNode physicsSphere=new PhysicsNode(sphere, CollisionShape.ShapeTypes.SPHERE);
```
## Adding physics objects to the PhysicsSpace ##
In order for PhysicsNodes to work, they need to be added to the PhysicsSpace and to the jme scenegraph.
```
//Add sphere to physics space
PhysicsSpace.getPhysicsSpace().add(physicsSphere);
//Add sphere to jme scenegraph
getRootNode().attachChild(physicsSphere);
physicsSphere.updateRenderState();
```

## Notes ##
Normally, the localTranslation/Rotation of the attached spatial should always be zero and not be changed, the PhysicsNode should be rotated and translated instead. You can however move/rotate or animate the child spatial for some effect but you have to be aware that the actual physics shape stays in its place/does not change.

PhysicsNodes with mass=0 are static and do not move.

To shift the center of mass see [CenterOfMass](CenterOfMass.md)

It is not recommended to have PhysicsNodes as child nodes of other PhysicsNodes

getLocalTranslation().set() does not set the physics object location, use setLocalTranslation(), same applies for getLocalRotation()

To get informed about collisions of PhysicsNodes, see CollisionEvents
## JavaDoc ##
http://jbullet-jme.googlecode.com/svn/trunk/jbullet-jme/javadoc/com/jmex/jbullet/nodes/PhysicsNode.html