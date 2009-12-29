/*
 * Copyright (c) 2009 Normen Hansen
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 *
 * * Neither the name of 'Normen Hansen' nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.jmex.jbullet.nodes;

import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.dynamics.vehicle.DefaultVehicleRaycaster;
import com.bulletphysics.dynamics.vehicle.RaycastVehicle;
import com.bulletphysics.dynamics.vehicle.VehicleRaycaster;
import com.bulletphysics.dynamics.vehicle.VehicleTuning;
import com.bulletphysics.dynamics.vehicle.WheelInfo;
import com.jme.math.Quaternion;
import com.jme.math.Vector3f;
import com.jme.scene.Spatial;
import com.jmex.jbullet.PhysicsSpace;
import com.jmex.jbullet.collision.CollisionShape;
import com.jmex.jbullet.collision.CollisionShape.Shapes;
import com.jmex.jbullet.util.Converter;
import java.util.LinkedList;
import java.util.List;


/**
 * <p>PhysicsVehicleNode - Special PhysicsNode that implements vehicle functions</p>
 * <p>
 * <i>From bullet manual:</i><br>
 * For most vehicle simulations, it is recommended to use the simplified Bullet
 * vehicle model as provided in btRaycastVehicle. Instead of simulation each wheel
 * and chassis as separate rigid bodies, connected by constraints, it uses a simplified model.
 * This simplified model has many benefits, and is widely used in commercial driving games.<br>
 * The entire vehicle is represented as a single rigidbody, the chassis.
 * The collision detection of the wheels is approximated by ray casts,
 * and the tire friction is a basic anisotropic friction model.
 * </p>
 * <p>
 * USAGE:<br>
 * A PhysicsVehicleNode can be used like a normal PhysicsNode but has some specialties:<br>
 * You can add wheels to the vehicle by calling the addWheel() method. The given jme
 * spatial will be displayed in the wheels location and rotate accordingly.<br>
 * You can accelerate/brake/steer the car with the corresponding methods.
 * </p>
 * TODO:<br>
 * - wheel rotation weirdness<br>
 * - accelerate/brake single<br>
 * - aceelerate/brake groups (front wheel)<br>
 * </p>
 * @see com.jmex.jbullet.node.PhysicsNode
 * @see com.jmex.jbullet.PhysicsSpace
 * @author normenhansen
 */
public class PhysicsVehicleNode extends PhysicsNode{
    private RaycastVehicle vehicle;
    private VehicleTuning tuning;
    private VehicleRaycaster rayCaster;
    private List<Spatial> wheels=new LinkedList<Spatial>();
    private boolean updateVehicle=false;
    private float rollInfluence=1.0f;
    private float frictionSlip=10.5f;
    //standards:
//	public float suspensionStiffness = 5.88f;
//	public float suspensionCompression = 0.83f;
//	public float suspensionDamping = 0.88f;
//	public float maxSuspensionTravelCm = 500f;
//	public float frictionSlip = 10.5f;
    private float maxSuspensionTravelCm=500f;
    private float suspensionCompression=4.4f;
    private float suspensionDamping=2.3f;
    private float suspensionStiffness=20.0f;

    private float engineForce=0.0f;
    private boolean applyEngineForce=false;
    private float brakeValue=0.0f;
    private boolean applyBrake=false;

    private float steerValue=0.0f;
    private boolean applySteer=false;

    public PhysicsVehicleNode(Spatial child){
        super(child, Shapes.BOX);
    }

    public PhysicsVehicleNode(Spatial child, int collisionShapeType){
        super(child, collisionShapeType);
    }

    public PhysicsVehicleNode(Spatial child, int collisionShapeType, float mass){
        super(child, collisionShapeType, mass);
    }

    public PhysicsVehicleNode(Spatial child, CollisionShape shape){
        super(child, shape);
    }

    public PhysicsVehicleNode(Spatial child, CollisionShape shape, float mass){
        super(child, shape, mass);
    }

    /**
     * TODO:<br>
     * - rebuild vehicle<br>
     */
    @Override
    protected void updateRigidBody() {
        super.updateRigidBody();
        rBody.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
        if(tuning==null)
            createVehicleConstraint();
        else{
            updateVehicleConstraint();
        }
    }

    private void createVehicleConstraint() {

        tuning=new VehicleTuning();
        updateVehicleConstraint();
        rayCaster=new DefaultVehicleRaycaster(PhysicsSpace.getPhysicsSpace().getDynamicsWorld());
        vehicle=new RaycastVehicle(tuning, rBody, rayCaster);

    }

    private void updateVehicleConstraint() {
        tuning.frictionSlip=frictionSlip;
        tuning.maxSuspensionTravelCm=maxSuspensionTravelCm;
        tuning.suspensionCompression=suspensionCompression;
        tuning.suspensionDamping=suspensionDamping;
        tuning.suspensionStiffness=suspensionStiffness;
        if(vehicle!=null&&vehicle.wheelInfo!=null)
        for(WheelInfo wheel:vehicle.wheelInfo){
            wheel.suspensionStiffness = suspensionStiffness;
            wheel.wheelsDampingRelaxation = suspensionDamping;
            wheel.wheelsDampingCompression = suspensionCompression;
            wheel.frictionSlip = frictionSlip;
            wheel.rollInfluence = rollInfluence;
            wheel.maxSuspensionTravelCm = maxSuspensionTravelCm;
        }
    }

    /**
     * add a wheel to this vehicle
     * @param spat the wheel Spatial (mesh)
     * @param connectionPoint the location of the wheel relative to the car
     * @param direction the direction of the wheel (should be -Y / -1,0,0 for a normal car)
     * @param axle The axis of the wheel (should be -X / 0,-1,0 for a normal car)
     * @param suspensionRestLength the suspension rest length
     * @param wheelRadius the wheel radius
     * @param isFrontWheel sets if this wheel is a front wheel (steering)
     */
    public void addWheel(Spatial spat, Vector3f connectionPoint, Vector3f direction, Vector3f axle, float suspensionRestLength, float wheelRadius, boolean isFrontWheel){
        wheels.add(spat);
        this.attachChild(spat);
        vehicle.addWheel(Converter.convert(connectionPoint), Converter.convert(direction), Converter.convert(axle), suspensionRestLength, wheelRadius, tuning, isFrontWheel);
        updateVehicleConstraint();
        for (int i = 0; i < wheels.size(); i++) {
            Spatial spatial=wheels.get(i);
            Converter.convert(vehicle.wheelInfo.get(i).worldTransform.origin,tempLocation);
            Converter.convert(vehicle.wheelInfo.get(i).worldTransform.basis,tempMatrix);

            //SET LOCATION
            spatial.getLocalTranslation().set( tempLocation ).subtractLocal( getWorldTranslation() );
            spatial.getLocalTranslation().divideLocal( getWorldScale() );
            tempRotation.set( getWorldRotation()).inverseLocal().multLocal( spatial.getLocalTranslation() );

            //SET ROTATION
            tempRotation2.fromRotationMatrix(tempMatrix);
            Quaternion myRot=spatial.getLocalRotation();
            tempRotation.set(getWorldRotation()).inverseLocal().mult(tempRotation2,myRot);

        }
    }

    /**
     * @return the frictionSlip
     */
    public float getFrictionSlip() {
        return frictionSlip;
    }

    /**
     * Sets the friction of the wheels
     * @param frictionSlip the frictionSlip to set
     */
    public void setFrictionSlip(float frictionSlip) {
        this.frictionSlip = frictionSlip;
        updateVehicle=true;
    }

    /**
     * @return the rollInfluence
     */
    public float getRollInfluence() {
        return rollInfluence;
    }

    /**
     * Sets how much wheels without motor influence the vehicles path (e.g. front
     * steering, rear acceleration)<br>
     * default=1.0f
     * @param rollInfluence the rollInfluence to set
     */
    public void setRollInfluence(float rollInfluence) {
        this.rollInfluence = rollInfluence;
        updateVehicle=true;
    }

    /**
     * @return the maxSuspensionTravelCm
     */
    public float getMaxSuspensionTravelCm() {
        return maxSuspensionTravelCm;
    }

    /**
     * this is (as far as I understand bullet) the size of the plane where the wheel touches
     * the floor. Set this too low and your car will fall through the ground..
     * @param maxSuspensionTravelCm the maxSuspensionTravelCm to set
     */
    public void setMaxSuspensionTravelCm(float maxSuspensionTravelCm) {
        this.maxSuspensionTravelCm = maxSuspensionTravelCm;
        updateVehicle=true;
    }

    /**
     * @return the suspensionCompression
     */
    public float getSuspensionCompression() {
        return suspensionCompression;
    }

    /**
     * @param suspensionCompression the suspensionCompression to set
     */
    public void setSuspensionCompression(float suspensionCompression) {
        this.suspensionCompression = suspensionCompression;
        updateVehicle=true;
    }

    /**
     * @return the suspensionDamping
     */
    public float getSuspensionDamping() {
        return suspensionDamping;
    }

    /**
     * @param suspensionDamping the suspensionDamping to set
     */
    public void setSuspensionDamping(float suspensionDamping) {
        this.suspensionDamping = suspensionDamping;
        updateVehicle=true;
    }

    /**
     * @return the suspensionStiffness
     */
    public float getSuspensionStiffness() {
        return suspensionStiffness;
    }

    /**
     * @param suspensionStiffness the suspensionStiffness to set
     */
    public void setSuspensionStiffness(float suspensionStiffness) {
        this.suspensionStiffness = suspensionStiffness;
        updateVehicle=true;
    }

    /**
     * apply the given engine force
     * @param force
     * @param apply
     */
    public void accelerate(float force, boolean apply){
        this.engineForce=force;
        //TODO: applyEngineForce always true..
        applyEngineForce=apply;
        applyBrake=false;
    }

    /**
     * set the given steering value (0 = forward)
     * @param value the steering angle of the front wheels (Pi = 360deg)
     */
    public void steer(float value){
        steerValue=value;
        applySteer=true;
    }

    public void brake(float value){
        brake(value,true);
    }

    public void brake(float value, boolean apply){
        brakeValue=value;
        applyBrake=apply;
    }

    /**
     * @return the vehicle
     */
    public RaycastVehicle getVehicle() {
        return vehicle;
    }

    @Override
    public void syncPhysics() {
        if(updateVehicle){
            updateVehicleConstraint();
        }
        if(applyEngineForce){
            for (int i = 0; i < vehicle.wheelInfo.size(); i++) {
                WheelInfo wheelInfo = vehicle.wheelInfo.get(i);
                if(!wheelInfo.bIsFrontWheel){
                    vehicle.applyEngineForce(engineForce, i);
                }
                vehicle.setBrake(0.0f, i);
            }
        }
        if(applyBrake){
            for (int i = 0; i < vehicle.wheelInfo.size(); i++) {
                WheelInfo wheelInfo = vehicle.wheelInfo.get(i);
                if(!wheelInfo.bIsFrontWheel){
                    vehicle.applyEngineForce(0.0f, i);
                }
                vehicle.setBrake(brakeValue, i);
            }
        }
        if(applySteer){
            for (int i = 0; i < vehicle.wheelInfo.size(); i++) {
                WheelInfo wheelInfo = vehicle.wheelInfo.get(i);
                if(wheelInfo.bIsFrontWheel)
                    vehicle.setSteeringValue(steerValue, i);
            }
            applySteer=false;
        }
        super.syncPhysics();


        for (int i = 0; i < wheels.size(); i++) {
            Spatial spatial=wheels.get(i);
            Converter.convert(vehicle.wheelInfo.get(i).worldTransform.origin,tempLocation);

            //SET LOCATION
            spatial.getLocalTranslation().set( tempLocation ).subtractLocal( getWorldTranslation() );
            spatial.getLocalTranslation().divideLocal( getWorldScale() );
            tempRotation.set(getWorldRotation()).inverseLocal().multLocal( spatial.getLocalTranslation() );

            Converter.convert(vehicle.wheelInfo.get(i).worldTransform.basis,tempMatrix);
            //SET ROTATION
            tempRotation2.fromRotationMatrix(tempMatrix);
            tempRotation.set(getWorldRotation()).inverseLocal().mult(tempRotation2,spatial.getLocalRotation());

        }
    }

}
