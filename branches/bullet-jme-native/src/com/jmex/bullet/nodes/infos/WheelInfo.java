/*
 * Copyright (c) 2009-2010 Normen Hansen
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
package com.jmex.bullet.nodes.infos;

import com.jme.math.Quaternion;
import com.jme.math.Vector3f;
import com.jme.scene.Spatial;
import com.jmex.bullet.nodes.PhysicsVehicleNode;
//import com.jmex.jbullet.util.Converter;

/**
 * Stores info about one wheel of a PhysicsVehicleNode
 * @author normenhansen
 */
public class WheelInfo {
    private PhysicsVehicleNode parent;
//    private com.bulletphysics.dynamics.vehicle.WheelInfo wheelInfo;
    private Spatial spatial;
    private boolean frontWheel;
    private Vector3f location=new Vector3f();
    private Vector3f direction=new Vector3f();
    private Vector3f axle=new Vector3f();

    private float suspensionStiffness = 20.0f;
    private float wheelsDampingRelaxation = 2.3f;
    private float wheelsDampingCompression = 4.4f;
    private float frictionSlip = 10.5f;
    private float rollInfluence = 1.0f;
    private float maxSuspensionTravelCm = 500f;

    private float radius;
    private float restLength;

    private float steerValue=0;
    private float engineForce=0;
    private float brakeForce=0;

    private com.jme.math.Vector3f tempLocation=new com.jme.math.Vector3f();
    private com.jme.math.Quaternion tempRotation=new com.jme.math.Quaternion();
    private com.jme.math.Quaternion tempRotation2=new com.jme.math.Quaternion();
    private com.jme.math.Matrix3f tempMatrix=new com.jme.math.Matrix3f();
    
    public WheelInfo(PhysicsVehicleNode parent, Spatial spat, Vector3f location, Vector3f direction, Vector3f axle,
            float restLength, float radius, boolean frontWheel) {
        this.parent=parent;
        this.spatial = spat;
        this.location.set(location);
        this.direction.set(direction);
        this.axle.set(axle);
        this.frontWheel=frontWheel;
        this.restLength=restLength;
        this.radius=radius;
    }

    public PhysicsVehicleNode getParent() {
        return parent;
    }

    public void setParent(PhysicsVehicleNode parent) {
        this.parent = parent;
    }

//    public com.bulletphysics.dynamics.vehicle.WheelInfo getWheelInfo() {
//        return wheelInfo;
//    }
//
//    public void setWheelInfo(com.bulletphysics.dynamics.vehicle.WheelInfo wheelInfo) {
//        this.wheelInfo = wheelInfo;
//    }

    public Spatial getSpat() {
        return spatial;
    }

    public void setSpat(Spatial spat) {
        this.spatial = spat;
    }

    public boolean isFrontWheel() {
        return frontWheel;
    }

    public void setFrontWheel(boolean frontWheel) {
        this.frontWheel = frontWheel;
        applyInfo();
    }

    public Vector3f getLocation() {
        return location;
    }

    public void setLocation(Vector3f location) {
        this.location = location;
        applyInfo();
    }

    public Vector3f getDirection() {
        return direction;
    }

    public void setDirection(Vector3f direction) {
        this.direction = direction;
        applyInfo();
    }

    public Vector3f getAxle() {
        return axle;
    }

    public void setAxle(Vector3f axle) {
        this.axle = axle;
        applyInfo();
    }

    public float getSuspensionStiffness() {
        return suspensionStiffness;
    }

    public void setSuspensionStiffness(float suspensionStiffness) {
        this.suspensionStiffness = suspensionStiffness;
        applyInfo();
    }

    public float getWheelsDampingRelaxation() {
        return wheelsDampingRelaxation;
    }

    public void setWheelsDampingRelaxation(float wheelsDampingRelaxation) {
        this.wheelsDampingRelaxation = wheelsDampingRelaxation;
        applyInfo();
    }

    public float getWheelsDampingCompression() {
        return wheelsDampingCompression;
    }

    public void setWheelsDampingCompression(float wheelsDampingCompression) {
        this.wheelsDampingCompression = wheelsDampingCompression;
        applyInfo();
    }

    public float getFrictionSlip() {
        return frictionSlip;
    }

    public void setFrictionSlip(float frictionSlip) {
        this.frictionSlip = frictionSlip;
        applyInfo();
    }

    public float getRollInfluence() {
        return rollInfluence;
    }

    /**
     * Sets how much wheels without motor influence the vehicles path (e.g. front
     * steering, rear acceleration)<br>
     * default=1.0f<br>
     * Use before adding wheels, this is the default used when adding wheels.
     * After adding the wheel, use direct wheel access.
     * @param rollInfluence the rollInfluence to set
     */
    public void setRollInfluence(float rollInfluence) {
        this.rollInfluence = rollInfluence;
        applyInfo();
    }

    public float getMaxSuspensionTravelCm() {
        return maxSuspensionTravelCm;
    }

    public void setMaxSuspensionTravelCm(float maxSuspensionTravelCm) {
        this.maxSuspensionTravelCm = maxSuspensionTravelCm;
        applyInfo();
    }
    
    public void applyInfo(){
//        wheelInfo.suspensionStiffness = suspensionStiffness;
//        wheelInfo.wheelsDampingRelaxation = wheelsDampingRelaxation;
//        wheelInfo.wheelsDampingCompression = wheelsDampingCompression;
//        wheelInfo.frictionSlip = frictionSlip;
//        wheelInfo.rollInfluence = rollInfluence;
//        wheelInfo.maxSuspensionTravelCm = maxSuspensionTravelCm;
//        wheelInfo.wheelsRadius = radius;
//        wheelInfo.bIsFrontWheel = frontWheel;
    }

    public void syncPhysics(){
//        parent.getVehicle().updateWheelTransformsWS(wheelInfo,true);
//        parent.getVehicle().getWheelTransformWS(wheelIndex, tempTrans);

//        Converter.convert(wheelInfo.worldTransform.origin,tempLocation);
//        Converter.convert(wheelInfo.worldTransform.basis,tempMatrix);
        //SET LOCATION
        spatial.getLocalTranslation().set( tempLocation ).subtractLocal( parent.getWorldTranslation() );
        spatial.getLocalTranslation().divideLocal( parent.getWorldScale() );
        tempRotation.set( parent.getWorldRotation()).inverseLocal().multLocal( spatial.getLocalTranslation() );

        //SET ROTATION
        tempRotation2.fromRotationMatrix(tempMatrix);
        Quaternion myRot=spatial.getLocalRotation();
        tempRotation.set(parent.getWorldRotation()).inverseLocal().mult(tempRotation2,myRot);
    }

    public float getRadius() {
        return radius;
    }

    public void setRadius(float radius) {
        this.radius = radius;
        applyInfo();
    }

    public float getRestLength() {
        return restLength;
    }

    public void setRestLength(float restLength) {
        this.restLength = restLength;
        applyInfo();
    }

    public float getSteerValue() {
        return steerValue;
    }

    public void setSteerValue(float steerValue) {
        this.steerValue = steerValue;
        applyInfo();
    }

    public float getEngineForce() {
        return engineForce;
    }

    public void setEngineForce(float engineForce) {
        this.engineForce = engineForce;
        applyInfo();
    }

    public float getBrakeForce() {
        return brakeForce;
    }

    public void setBrakeForce(float brakeForce) {
        this.brakeForce = brakeForce;
        applyInfo();
    }

}
