## Introduction ##
To receive information about collisions happening in the PhysicsSpace, your class has to implement the CollisionListener interface and the collision(CollisionEvent event) method.

```
public class MyClass implements CollisionListener {
    public void collision(CollisionEvent event){
        //get collision force
        float force=event.getAppliedImpulse();
    }
}
```

Then you register your CollisionListener at the PhysicsSpace to receive CollisionEvents.
```
PhysicsSpace.getPhysicsSpace.addCollisionListener(new MyClass());
```
You will receive a CollisionEvent object on every collision that happens in the PhysicsSpace containing all information about the collision.

## JavaDoc ##
http://jbullet-jme.googlecode.com/svn/trunk/jbullet-jme/javadoc/com/jmex/jbullet/collision/CollisionListener.html

http://jbullet-jme.googlecode.com/svn/trunk/jbullet-jme/javadoc/com/jmex/jbullet/collision/CollisionEvent.html