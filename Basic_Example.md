**api may change until release**
## Create physics space ##
Create a PhysicsSpace like this:
```
PhysicsSpace pSpace=PhysicsSpace.getPhysicsSpace();
```
## Add physics objects ##
Now you can add physics objects to the world like this:

```
//Create a jme sphere
Sphere sphere=new Sphere("physicssphere",16,16,1f);

//Create physics sphere from it
PhysicsNode physicsSphere=new PhysicsNode(sphere, CollisionShape.ShapeTypes.SPHERE);
physicsSphere.setLocalTranslation(new Vector3f(3,6,0));

//Add sphere to jme scenegraph
getRootNode().attachChild(physicsSphere);
physicsSphere.updateRenderState();

//Add sphere to physics space
pSpace.add(physicsSphere);

//Add a physics mesh as floor to the world, note that objects with mass=0 are static
Box box=new Box("physicsfloor",Vector3f.ZERO,100f,0.2f,100f);
PhysicsNode floor=new PhysicsNode(box, CollisionShape.ShapeTypes.MESH);
floor.setMass(0);
floor.setLocalTranslation(new Vector3f(0f,-6,0f));

//Add floor to jme scenegraph
getRootNode().attachChild(floor);
floor.updateRenderState();

//Add floor to physics space
pSpace.add(floor);
```
## Update call ##
In your jme2 Gamestate, update physics like this:
```
public void update(float tpf) {
    super.update(tpf);
    pSpace.update(tpf);
}
```

## Examples in the source ##
Theres more examples of jbullet functions in the source of jbullet-jme here:

http://jbullet-jme.googlecode.com/svn/trunk/jbullet-jme/src/jmetest/jbullet/