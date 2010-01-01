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

import com.jme.math.Vector3f;
import com.jme.util.GameTaskQueue;
import com.jme.util.GameTaskQueueManager;
import com.jmex.jbullet.util.Converter;
import java.util.concurrent.Callable;

/**
 *
 * @author normenhansen
 */
public class TranslationalLimitMotor {
    private com.bulletphysics.dynamics.constraintsolver.TranslationalLimitMotor motor;

	private Vector3f lowerLimit = new Vector3f(); //! Bounce parameter for linear limit
    private Vector3f upperLimit = new Vector3f();
    private Vector3f accumulatedImpulse = new Vector3f();
    private float limitSoftness;
    private float damping;
    private float restitution;

    protected GameTaskQueue pQueue=GameTaskQueueManager.getManager().getQueue("jbullet_sync");

    public TranslationalLimitMotor(com.bulletphysics.dynamics.constraintsolver.TranslationalLimitMotor motor) {
        this.motor = motor;
        this.limitSoftness=motor.limitSoftness;
        this.damping=motor.damping;
        this.restitution=motor.restitution;
        Converter.convert(motor.lowerLimit,lowerLimit);
        Converter.convert(motor.upperLimit,upperLimit);
        Converter.convert(motor.accumulatedImpulse,accumulatedImpulse);
    }

    public com.bulletphysics.dynamics.constraintsolver.TranslationalLimitMotor getMotor() {
        return motor;
    }

    public Vector3f getLowerLimit() {
        return lowerLimit;
    }

    public void setLowerLimit(Vector3f lowerLimit) {
        this.lowerLimit.set(lowerLimit);
        pQueue.enqueue(doSyncPhysics);
    }

    public Vector3f getUpperLimit() {
        return upperLimit;
    }

    public void setUpperLimit(Vector3f upperLimit) {
        this.upperLimit.set(upperLimit);
        pQueue.enqueue(doSyncPhysics);
    }

    public Vector3f getAccumulatedImpulse() {
        return accumulatedImpulse;
    }

    public void setAccumulatedImpulse(Vector3f accumulatedImpulse) {
        this.accumulatedImpulse.set(accumulatedImpulse);
        pQueue.enqueue(doSyncPhysics);
    }

    public float getLimitSoftness() {
        return limitSoftness;
    }

    public void setLimitSoftness(float limitSoftness) {
        this.limitSoftness = limitSoftness;
        pQueue.enqueue(doSyncPhysics);
    }

    public float getDamping() {
        return damping;
    }

    public void setDamping(float damping) {
        this.damping = damping;
         pQueue.enqueue(doSyncPhysics);
   }

    public float getRestitution() {
        return restitution;
    }

    public void setRestitution(float restitution) {
        this.restitution = restitution;
        pQueue.enqueue(doSyncPhysics);
    }

    public void syncPhysics() {
        motor.limitSoftness=this.limitSoftness;
        motor.damping=this.damping;
        motor.restitution=this.restitution;
        Converter.convert(this.lowerLimit,motor.lowerLimit);
        Converter.convert(this.upperLimit,motor.upperLimit);
        Converter.convert(this.accumulatedImpulse,motor.accumulatedImpulse);
    }

    private Callable doSyncPhysics=new Callable(){
        public Object call() throws Exception {
            syncPhysics();
            return null;
        }
    };

}
