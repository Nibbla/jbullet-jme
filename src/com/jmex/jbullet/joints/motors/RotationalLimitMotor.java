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

import java.io.IOException;

import com.jme.math.Vector3f;
import com.jme.util.export.InputCapsule;
import com.jme.util.export.JMEExporter;
import com.jme.util.export.JMEImporter;
import com.jme.util.export.OutputCapsule;
import com.jme.util.export.Savable;
import com.jme.util.export.binary.BinaryLoaderModule;

/**
 *
 * @author normenhansen
 */
public class RotationalLimitMotor implements Savable {
    private com.bulletphysics.dynamics.constraintsolver.RotationalLimitMotor motor;

    public RotationalLimitMotor(com.bulletphysics.dynamics.constraintsolver.RotationalLimitMotor motor) {
        this.motor=motor;
    }

    public com.bulletphysics.dynamics.constraintsolver.RotationalLimitMotor getMotor() {
        return motor;
    }

    public float getLoLimit() {
        return motor.loLimit;
    }

    public void setLoLimit(float loLimit) {
        motor.loLimit = loLimit;
    }

    public float getHiLimit() {
        return motor.hiLimit;
    }

    public void setHiLimit(float hiLimit) {
        motor.hiLimit = hiLimit;
    }

    public float getTargetVelocity() {
        return motor.targetVelocity;
    }

    public void setTargetVelocity(float targetVelocity) {
        motor.targetVelocity = targetVelocity;
    }

    public float getMaxMotorForce() {
        return motor.maxMotorForce;
    }

    public void setMaxMotorForce(float maxMotorForce) {
        motor.maxMotorForce = maxMotorForce;
    }

    public float getMaxLimitForce() {
        return motor.maxLimitForce;
    }

    public void setMaxLimitForce(float maxLimitForce) {
        motor.maxLimitForce = maxLimitForce;
    }

    public float getDamping() {
        return motor.damping;
    }

    public void setDamping(float damping) {
        motor.damping = damping;
    }

    public float getLimitSoftness() {
        return motor.limitSoftness;
    }

    public void setLimitSoftness(float limitSoftness) {
        motor.limitSoftness = limitSoftness;
    }

    public float getERP() {
        return motor.ERP;
    }

    public void setERP(float ERP) {
        motor.ERP = ERP;
    }

    public float getBounce() {
        return motor.bounce;
    }

    public void setBounce(float bounce) {
        motor.bounce = bounce;
    }

    public boolean isEnableMotor() {
        return motor.enableMotor;
    }

    public void setEnableMotor(boolean enableMotor) {
        motor.enableMotor = enableMotor;
    }

	@Override
	public Class getClassTag() {
		return this.getClass();
	}

	@Override
	public void read(JMEImporter arg0) throws IOException {
		InputCapsule capsule = arg0.getCapsule(this);
		if(motor==null){
			throw new NullPointerException("Motor is Null.");
		}
		setDamping(capsule.readFloat("damping", 0));
		setLimitSoftness(capsule.readFloat("limitSoftness", 0));
		setBounce(capsule.readFloat("bounce", 0));
		setERP(capsule.readFloat("ERP", 0));
		setHiLimit(capsule.readFloat("hiLimit", 0));
		setTargetVelocity(capsule.readFloat("targetVelocity", 0));
		setLoLimit(capsule.readFloat("loLimit", 0));
		setMaxLimitForce(capsule.readFloat("maxLimitForce", 0));
		setMaxMotorForce(capsule.readFloat("maxMotorForce", 0));
		setEnableMotor(capsule.readBoolean("enabled", false));
	}

	@Override
	public void write(JMEExporter e) throws IOException {
		OutputCapsule capsule = e.getCapsule(this);
		capsule.write(motor.damping, "damping", 0);
		capsule.write(motor.limitSoftness, "limitSoftness", 0);
		capsule.write(motor.bounce, "bounce", 0);
		capsule.write(motor.ERP, "ERP", 0);
		capsule.write(motor.hiLimit, "hiLimit", 0);
		capsule.write(motor.targetVelocity, "targetVelocity", 0);
		capsule.write(motor.loLimit, "loLimit", 0);
		capsule.write(motor.maxLimitForce, "maxLimitForce", 0);
		capsule.write(motor.maxMotorForce, "maxMotorForce", 0);
		capsule.write(isEnableMotor(), "enabled", false);
		
	}

	
    
}
