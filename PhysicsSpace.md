The PhysicsSpace defines the basic physics world in jbullet-jme. Things such as global gravity etc. are set here and all [PhysicsNode](PhysicsNode.md)s are added here.

## Creating the PhysicsSpace ##
The PhysicsSpace needs to be started by getting a PhysicsSpace Object with PhysicsSpace.getPhysicsSpace().
```
PhysicsSpace pSpace=PhysicsSpace.getPhysicsSpace();
```
You can add options to the getPhysicsSpace() method to improve the physics performance.
```
Vector3f worldMin=new Vector3f(-1000,-1000,-1000);
Vector3f worldMax=new Vector3f(1000,1000,1000);
PhysicsSpace pSpace=PhysicsSpace.getPhysicsSpace(worldMin, worldMax, 
                        PhysicsSpace.BroadphaseTypes.AXIS_SWEEP_3);
```
_Note: after you have created a PhysicsSpace, the extended getPhysicsSpace method does not change the settings of the PhysicsSpace, you have to call physicsSpace.destroy() before to create a new PhysicsSpace with new settings._

## Updating the PhysicsSpace ##
To update the physics space you need to call the update(float time) method in your jme gamestate.
```
public void update(float tpf){
    physicsSpace.update(tpf);
}
```
You can alter the physics speed by multiplying the supplied update time value with an arbitrary number (not recommended).

### Physics Settings ###
You can alter the physics accuracy with the physicsSpace.setAccuracy(float acc) method. The standard accuracy is 1f/60f (60 fps). Setting too low values results in, well.. inaccurate physics :)
```
//trading lower accuracy for higher performance
physicsSpace.setAccuracy(1f/30f);
```

## JavaDoc ##
http://jbullet-jme.googlecode.com/svn/trunk/jbullet-jme/javadoc/com/jmex/jbullet/PhysicsSpace.html