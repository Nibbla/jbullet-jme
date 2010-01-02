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
import com.bulletphysics.linearmath.MotionState;
import com.bulletphysics.linearmath.Transform;
import com.jme.math.Vector3f;
import com.jme.scene.Spatial;
import com.jmex.jbullet.PhysicsSpace;
import com.jmex.jbullet.collision.shapes.CollisionShape;
import com.jmex.jbullet.collision.shapes.CollisionShape.Shapes;
import com.jmex.jbullet.nodes.infos.WheelInfo;
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
 * @see com.jmex.jbullet.nodes.PhysicsNode
 * @see com.jmex.jbullet.PhysicsSpace
 * @author normenhansen
 */
public class PhysicsVehicleNode extends PhysicsNode{
    private RaycastVehicle vehicle;
    private VehicleTuning tuning;
    private VehicleRaycaster rayCaster;
    private List<WheelInfo> wheels=new LinkedList<WheelInfo>();

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

    @Override
    protected MotionState createMotionState(){
        return new MotionState(){

            public Transform getWorldTransform(Transform out) {
                if(out==null)
                    out=new Transform();

                tempRotation.set(getWorldRotation());
                Converter.convert(tempRotation, tempRot);

                out.basis.set(tempRot);
                out.origin.set(Converter.convert(getWorldTranslation()));
                return out;
            }

            public void setWorldTransform(Transform worldTrans) {
                motionStateTrans.set(worldTrans);
                applyMotionState();
            }

        };
    }

    private void applyMotionState() {
        Converter.convert(motionStateTrans.origin,tempLocation);
        setWorldTranslation(tempLocation);

        Converter.convert(motionStateTrans.basis,tempMatrix);
        tempRotation.fromRotationMatrix(tempMatrix);
        setWorldRotation(tempRotation);

        //to set wheel locations
        syncWheels();
    }

    @Override
    protected void postRebuild(){
        super.postRebuild();
        createVehicleConstraint();
        rBody.setActivationState(CollisionObject.DISABLE_DEACTIVATION);
    }

    private void createVehicleConstraint() {
        if(tuning==null)
            tuning=new VehicleTuning();
        rayCaster=new DefaultVehicleRaycaster(PhysicsSpace.getPhysicsSpace().getDynamicsWorld());
        vehicle=new RaycastVehicle(tuning, rBody, rayCaster);
        if(wheels!=null)
        for(WheelInfo wheel:wheels){
            wheel.setWheelInfo(vehicle.addWheel(Converter.convert(wheel.getLocation()), Converter.convert(wheel.getDirection()), Converter.convert(wheel.getAxle()),
                    wheel.getRestLength(), wheel.getRadius(), tuning, wheel.isFrontWheel()));
            wheel.syncPhysics();
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
        this.attachChild(spat);
        WheelInfo info=new WheelInfo(this,spat,connectionPoint,direction,axle,suspensionRestLength,wheelRadius,isFrontWheel);
        info.setWheelInfo(
                vehicle.addWheel(Converter.convert(connectionPoint), Converter.convert(direction), Converter.convert(axle), suspensionRestLength, wheelRadius, tuning, isFrontWheel)
                );
        info.applyInfo();
        wheels.add(info);
        for (int i = 0; i < wheels.size(); i++) {
            wheels.get(i).syncPhysics();
        }
    }

    /**
     * @return the frictionSlip
     */
    public float getFrictionSlip() {
        return tuning.frictionSlip;
    }

    /**
     * Sets the friction of the wheels.
     * Use before adding wheels, this is the default used when adding wheels.
     * After adding the wheel, use direct wheel access.
     * @param frictionSlip the frictionSlip to set
     */
    public void setFrictionSlip(float frictionSlip) {
        tuning.frictionSlip = frictionSlip;
    }

    public void setFrictionSlip(int wheel, float frictionSlip) {
        wheels.get(wheel).setFrictionSlip(frictionSlip);
    }

    public void setRollInfluence(int wheel, float rollInfluence) {
        wheels.get(wheel).setRollInfluence(rollInfluence);
    }

    /**
     * @return the maxSuspensionTravelCm
     */
    public float getMaxSuspensionTravelCm() {
        return tuning.maxSuspensionTravelCm;
    }

    /**
     * this is (as far as I understand bullet) the size of the plane where the wheel touches
     * the floor. Set this too low and your car will fall through the ground..<br>
     * Use before adding wheels, this is the default used when adding wheels.
     * After adding the wheel, use direct wheel access.
     * @param maxSuspensionTravelCm the maxSuspensionTravelCm to set
     */
    public void setMaxSuspensionTravelCm(float maxSuspensionTravelCm) {
        tuning.maxSuspensionTravelCm = maxSuspensionTravelCm;
    }

    public void setMaxSuspensionTravelCm(int wheel, float maxSuspensionTravelCm) {
        wheels.get(wheel).setMaxSuspensionTravelCm(maxSuspensionTravelCm);
    }

    /**
     * @return the suspensionCompression
     */
    public float getSuspensionCompression() {
        return tuning.suspensionCompression;
    }

    /**
     * Use before adding wheels, this is the default used when adding wheels.
     * After adding the wheel, use direct wheel access.
     * @param suspensionCompression the suspensionCompression to set
     */
    public void setSuspensionCompression(float suspensionCompression) {
        tuning.suspensionCompression = suspensionCompression;
    }

    public void setSuspensionCompression(int wheel, float suspensionCompression) {
        wheels.get(wheel).setWheelsDampingCompression(suspensionCompression);
    }

    /**
     * @return the suspensionDamping
     */
    public float getSuspensionDamping() {
        return tuning.suspensionDamping;
    }

    /**
     * Use before adding wheels, this is the default used when adding wheels.
     * After adding the wheel, use direct wheel access.
     * @param suspensionDamping the suspensionDamping to set
     */
    public void setSuspensionDamping(float suspensionDamping) {
        tuning.suspensionDamping = suspensionDamping;
    }

    public void setSuspensionDamping(int wheel, float suspensionDamping) {
        wheels.get(wheel).setWheelsDampingRelaxation(suspensionDamping);
    }

    /**
     * @return the suspensionStiffness
     */
    public float getSuspensionStiffness() {
        return tuning.suspensionStiffness;
    }

    /**
     * Use before adding wheels, this is the default used when adding wheels.
     * After adding the wheel, use direct wheel access.
     * @param suspensionStiffness the suspensionStiffness to set
     */
    public void setSuspensionStiffness(float suspensionStiffness) {
        tuning.suspensionStiffness = suspensionStiffness;
    }

    public void setSuspensionStiffness(int wheel, float suspensionStiffness) {
        wheels.get(wheel).setSuspensionStiffness(suspensionStiffness);
    }

    /**
     * apply the given engine force
     * @param force
     */
    public void accelerate(float force){
        for(WheelInfo wheel:wheels){
            wheel.setEngineForce(force);
        }
        applyEngineForce();
    }

    public void accelerate(int wheelNumber, float force){
        WheelInfo wheelInfo=wheels.get(wheelNumber);
        wheelInfo.setEngineForce(force);
        applyEngineForce();
    }

    private void applyEngineForce(){
        for (int i = 0; i < wheels.size(); i++) {
            WheelInfo wheel=wheels.get(i);
            vehicle.applyEngineForce(wheel.getEngineForce(), i);
        }
    }


    /**
     * set the given steering value (0 = forward)
     * @param value the steering angle of the front wheels (Pi = 360deg)
     */
    public void steer(float value){
        for(WheelInfo wheel:wheels){
            if(wheel.isFrontWheel())
                wheel.setSteerValue(value);
        }
        applySteer();
    }

    private void applySteer() {
        for (int i = 0; i < wheels.size(); i++) {
            WheelInfo wheelInfo = wheels.get(i);
            vehicle.setSteeringValue(wheelInfo.getSteerValue(), i);
        }
    }

    public void brake(float value){
        for(WheelInfo wheel:wheels){
            wheel.setBrakeForce(value);
        }
        applyBrake();
    }

    public void brake(int wheelNumber, float force){
        WheelInfo wheelInfo=wheels.get(wheelNumber);
        wheelInfo.setBrakeForce(force);
        applyBrake();
    }

    private void applyBrake() {
        for (int i = 0; i < wheels.size(); i++) {
            WheelInfo wheelInfo = wheels.get(i);
            vehicle.setBrake(wheelInfo.getBrakeForce(), i);
        }
    }

    /**
     * used internally
     * @return the vehicle
     */
    public RaycastVehicle getVehicle() {
        return vehicle;
    }

    public void syncWheels() {
        if(wheels!=null)
        for (int i = 0; i < wheels.size(); i++) {
            wheels.get(i).syncPhysics();
        }
    }

}
