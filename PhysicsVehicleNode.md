The PhysicsVehicleNode is a special PhysicsNode that represents a vehicle in the physics space and can have wheels and be controlled via acceleration, steering etc.

## Creating a vehicle ##
Creating a basic Vehicle works like creating a PhysicsNode:
```
// Add a physics vehicle to the world
Box box=new Box("physicscar",Vector3f.ZERO,0.5f,0.5f,2f);
physicsCar=new PhysicsVehicleNode(box,CollisionShape.ShapeTypes.BOX);
```

## Adding wheels ##
When adding a wheel you have to specify the jme spatial that acts as the wheel, its translation in respect to the car and the axes, rest length and radius of the wheel.
```
Vector3f wheelDirection=new Vector3f(0,-1,0);
Vector3f wheelAxle=new Vector3f(-1,0,0);
// Create four wheels and add them at their locations
Sphere wheel=new Sphere("wheel",8,8,0.5f);
physicsCar.addWheel(wheel, new Vector3f(-1f,-0.5f,2f),
                wheelDirection, wheelAxle, 0.2f, 0.5f, true);
wheel=new Sphere("wheel",8,8,0.5f);
physicsCar.addWheel(wheel, new Vector3f(1f,-0.5f,2f),
                wheelDirection, wheelAxle, 0.2f, 0.5f, true);
wheel=new Sphere("wheel",8,8,0.5f);
physicsCar.addWheel(wheel, new Vector3f(-1f,-0.5f,-2f),
                wheelDirection, wheelAxle, 0.2f, 0.5f, false);
wheel=new Sphere("wheel",8,8,0.5f);
physicsCar.addWheel(wheel, new Vector3f(1f,-0.5f,-2f),
                wheelDirection, wheelAxle, 0.2f, 0.5f, false);
```

## Controlling the vehicle ##
The PhysicsVehicleNode has accelerate(float force) brake(float force) and steer(float angle) methods to control the vehicle. All of these work continuously, so they have to be set only once and the vehicle will accelerate/brake/steer until the value is changed. Single wheels can be accessed through the vehicle methods as well.

## Accessing wheel infos ##
To access a single wheels info, for example to get its current tracktion, you can use the physicsVehicleNode.getWheelInfo(int wheelNumber) method. You will get a WheelInfo object that you can use to get and set info on the wheel.
```
//play smoke animation at wheel contact location when wheel is sliding
WheelInfo wheelInfo=physicsCar.getWheelInfo(0);
if (wheelInfo.getSkidInfo() < 0.8f) {
    playSmokeAnimation(wheelInfo.getCollisionLocation());
}
```

## Notes ##
Infos mostly taken from:
[Some general notes on bullet vehicles by Kester Maddock](http://docs.google.com/Doc?docid=0AXVUZ5xw6XpKZGNuZG56a3FfMzU0Z2NyZnF4Zmo&hl=en&pli=1)
#### Raycast Vehicle ####
The ray cast vehicle consists works by casting a ray for each wheel.  Using the ray's intersection point, we can calculate the suspension length, and hence the suspension force.

The rays should originate inside the vehicle chassis's CollisionShape.  Otherwise, the wheel can be projected through world geometry.

Because rays are infinitely thin, it is possible for the wheel to fall through cracks in the geometry.

#### Shifting the center of mass ####
See [CenterOfMass](CenterOfMass.md)

#### Suspension ####
Suspension is provided by the spring force plus a damping force.  The damping force stops the car from bouncing forever.  There are two coefficients for damping: one for spring compression, and one for spring relaxation.  In a real vehicle, the compression damping is set much lower than the relaxation damping.  This means, when the vehicle hits a bump, it won't be transmitted to the chassis, resulting in a smooth ride.
Set the suspension damping as a fraction of critical damping: = k **2.0** btSqrt(m\_suspensionStiffness), where k is the proportion of critical damping.  For more information, see http://en.wikipedia.org/wiki/Damping
```
float stiffness=60.0f-200.0f;//200=f1 car
float compValue=0.0f-1.0f; //(lower than damp!)
float dampValue=0.0f-1.0f;
vehicle.setSuspensionCompression(compValue*2.0f*FastMath.sqrt(stiffness));
vehicle.setSuspensionDamping(dampValue*2.0f*FastMath.sqrt(stiffness));
vehicle.setSuspensionStiffness(stiffness);
```
#### Why are my wheels sinking through the ground? ####
The wheels sink through the ground when the suspension cannot support the weight of the vehicle.  You need to increase the suspension stiffness, max travel or suspension length.

## JavaDoc ##
http://jbullet-jme.googlecode.com/svn/trunk/jbullet-jme/javadoc/com/jmex/jbullet/nodes/PhysicsVehicleNode.html