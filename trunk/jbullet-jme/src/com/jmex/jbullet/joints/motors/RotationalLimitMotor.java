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
package com.jmex.jbullet.joints.motors;

import com.jme.util.GameTaskQueue;
import com.jme.util.GameTaskQueueManager;
import java.util.concurrent.Callable;

/**
 *
 * @author normenhansen
 */
public class RotationalLimitMotor {
    private com.bulletphysics.dynamics.constraintsolver.RotationalLimitMotor motor;
	private float loLimit;
    private float hiLimit;
    private float targetVelocity;
    private float maxMotorForce;
    private float maxLimitForce;
    private float damping;
    private float limitSoftness;
    private float ERP;
    private float bounce;
    private boolean enableMotor;
    
    protected GameTaskQueue pQueue=GameTaskQueueManager.getManager().getQueue("jbullet_sync");

    public RotationalLimitMotor(com.bulletphysics.dynamics.constraintsolver.RotationalLimitMotor motor) {
        this.motor=motor;
        this.loLimit=motor.loLimit;
        this.hiLimit=motor.hiLimit;
        this.targetVelocity=motor.targetVelocity;
        this.maxMotorForce=motor.maxMotorForce;
        this.maxLimitForce=motor.maxLimitForce;
        this.damping=motor.damping;
        this.limitSoftness=motor.limitSoftness;
        this.ERP=motor.ERP;
        this.bounce=motor.bounce;
        this.enableMotor=motor.enableMotor;
    }

    public com.bulletphysics.dynamics.constraintsolver.RotationalLimitMotor getMotor() {
        return motor;
    }

    public float getLoLimit() {
        return loLimit;
    }

    public void setLoLimit(float loLimit) {
        this.loLimit = loLimit;
        pQueue.enqueue(doSyncPhysics);
    }

    public float getHiLimit() {
        return hiLimit;
    }

    public void setHiLimit(float hiLimit) {
        this.hiLimit = hiLimit;
        pQueue.enqueue(doSyncPhysics);
    }

    public float getTargetVelocity() {
        return targetVelocity;
    }

    public void setTargetVelocity(float targetVelocity) {
        this.targetVelocity = targetVelocity;
        pQueue.enqueue(doSyncPhysics);
    }

    public float getMaxMotorForce() {
        return maxMotorForce;
    }

    public void setMaxMotorForce(float maxMotorForce) {
        this.maxMotorForce = maxMotorForce;
        pQueue.enqueue(doSyncPhysics);
    }

    public float getMaxLimitForce() {
        return maxLimitForce;
    }

    public void setMaxLimitForce(float maxLimitForce) {
        this.maxLimitForce = maxLimitForce;
        pQueue.enqueue(doSyncPhysics);
    }

    public float getDamping() {
        return damping;
    }

    public void setDamping(float damping) {
        this.damping = damping;
        pQueue.enqueue(doSyncPhysics);
    }

    public float getLimitSoftness() {
        return limitSoftness;
    }

    public void setLimitSoftness(float limitSoftness) {
        this.limitSoftness = limitSoftness;
        pQueue.enqueue(doSyncPhysics);
    }

    public float getERP() {
        return ERP;
    }

    public void setERP(float ERP) {
        this.ERP = ERP;
        pQueue.enqueue(doSyncPhysics);
    }

    public float getBounce() {
        return bounce;
    }

    public void setBounce(float bounce) {
        this.bounce = bounce;
        pQueue.enqueue(doSyncPhysics);
    }

    public boolean isEnableMotor() {
        return enableMotor;
    }

    public void setEnableMotor(boolean enableMotor) {
        this.enableMotor = enableMotor;
        pQueue.enqueue(doSyncPhysics);
    }
    
    public void syncPhysics() {
        motor.loLimit=this.loLimit;
        motor.hiLimit=this.hiLimit;
        motor.targetVelocity=this.targetVelocity;
        motor.maxMotorForce=this.maxMotorForce;
        motor.maxLimitForce=this.maxLimitForce;
        motor.damping=this.damping;
        motor.limitSoftness=this.limitSoftness;
        motor.ERP=this.ERP;
        motor.bounce=this.bounce;
        motor.enableMotor=this.enableMotor;
    }

    private Callable doSyncPhysics=new Callable(){
        public Object call() throws Exception {
            syncPhysics();
            return null;
        }
    };

}
