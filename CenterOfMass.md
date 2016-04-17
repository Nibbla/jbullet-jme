## Shifting the center of mass ##
If you want to shift the center of mass of a PhysicsNode or PhysicsVehicleNode, you will have to create a CompoundCollisionShape and attach the real CollisionShape for the PhysicsNode to it with an offset.
```
CompoundCollisionShape compoundCollisionShape=new CompoundCollisionShape();
BoxCollisionShape boxCollisionShape=new BoxCollisionShape(new Vector3f(0.5f,0.5f,0.5f));
compound.addChildShape(boxCollisionShape, new Vector3f(0,1,0));
```
![http://jbullet-jme.googlecode.com/svn/wiki/images/MassOffset.jpg](http://jbullet-jme.googlecode.com/svn/wiki/images/MassOffset.jpg)

In this example the effective center of mass of the BoxCollisionShape is shifted **down** 1 unit.

## Shifting the meshes accordingly ##
Since the center of mass is at the center of the PhysicsNode, you will have to shift the child spatial (e.g. mesh) that is belonging to the PhysicsNode as well to have it behave correctly.

```
mesh.setLocalTranslation(0,1,0);
PhysicsNode pNode=new PhysicsNode(mesh, compoundCollisionShape);
```
![http://jbullet-jme.googlecode.com/svn/wiki/images/MassOffset2.jpg](http://jbullet-jme.googlecode.com/svn/wiki/images/MassOffset2.jpg)