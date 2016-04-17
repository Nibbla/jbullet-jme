A CollisionShape defines the physics shape of a PhysicsNode, possible shapes are:

  * Box
  * Sphere
  * Capsule
  * Cylinder
  * Compound (combines shapes)
  * Mesh (world only)
  * GImpact

## From scratch ##
You can create a CollisionShape object and use that when constructing a PhysicsNode, this way it is possible to reuse that CollisionShape
```
BoxCollisionShape boxCollisionShape=new BoxCollisionShape(new Vector3f(0.5f,0.5f,0.5f));
PhysicsNode pNode=new PhysicsNode(mesh, boxCollisionShape);
```

## From BoundingVolume ##
In jbullet-jme, CollisionShapes can be drived from the jme BoundingVolume of a Spatial.
When you create a PhysicsNode with a spatial and shape type, the corresponding BoundingVolume is created on the PhysicsNode and a CollisionShape is automatically derived from that.
If you want the shape to be different than the auto-generated one, you can set a BoundingVolume to the PhysicsNode and call createCollisionShape() to recreate the corresponding CollisionShape.

## CompoundCollisionShape ##
A CompoundCollisionShape combines multiple other collision shapes to define more detailed collision shapes. See also [CenterOfMass](CenterOfMass.md)
```
CompoundCollisionShape compoundCollisionShape=new CompoundCollisionShape();
BoxCollisionShape boxCollisionShape=new BoxCollisionShape(new Vector3f(0.5f,0.5f,0.5f));
BoxCollisionShape boxCollisionShape2=new BoxCollisionShape(new Vector3f(2f,0.5f,2f));
compound.addChildShape(boxCollisionShape, new Vector3f(0,1,0));
compound.addChildShape(boxCollisionShape2, new Vector3f(0,0,0));
PhysicsNode pNode=new PhysicsNode(mesh, compoundCollisionShape);
```

## Notes ##
It is advised to reuse CollisionShapes as often as possible. It is not necessary to create a new collision shape for every 1x1x1 box in your world. You can use just one CollisionShape for multiple PhysicsNodes.

Currently, PhysicsNodes with mesh collision shape do not collide with other mesh PhysicsNodes, so mesh shapes are mostly to be used for static objects (floor/world) while box/sphere etc. shapes should be used for mobile objects. You can also use the GImpact shape type for mesh-accurate mobile objects but you will get lower performance.

## JavaDoc ##
http://jbullet-jme.googlecode.com/svn/trunk/jbullet-jme/javadoc/com/jmex/jbullet/collision/shapes/CollisionShape.html