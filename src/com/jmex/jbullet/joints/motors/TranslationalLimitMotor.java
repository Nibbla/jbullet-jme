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
import com.jmex.jbullet.util.Converter;

/**
 *
 * @author normenhansen
 */
public class TranslationalLimitMotor implements Savable {
    private com.bulletphysics.dynamics.constraintsolver.TranslationalLimitMotor motor;

    public TranslationalLimitMotor(com.bulletphysics.dynamics.constraintsolver.TranslationalLimitMotor motor) {
        this.motor = motor;
    }

    public com.bulletphysics.dynamics.constraintsolver.TranslationalLimitMotor getMotor() {
        return motor;
    }

    public Vector3f getLowerLimit() {
        return Converter.convert(motor.lowerLimit);
    }

    public void setLowerLimit(Vector3f lowerLimit) {
        Converter.convert(lowerLimit,motor.lowerLimit);
    }

    public Vector3f getUpperLimit() {
        return Converter.convert(motor.upperLimit);
    }

    public void setUpperLimit(Vector3f upperLimit) {
        Converter.convert(upperLimit,motor.upperLimit);
    }

    public Vector3f getAccumulatedImpulse() {
        return Converter.convert(motor.accumulatedImpulse);
    }

    public void setAccumulatedImpulse(Vector3f accumulatedImpulse) {
        Converter.convert(accumulatedImpulse,motor.accumulatedImpulse);
    }

    public float getLimitSoftness() {
        return motor.limitSoftness;
    }

    public void setLimitSoftness(float limitSoftness) {
        motor.limitSoftness = limitSoftness;
    }

    public float getDamping() {
        return motor.damping;
    }

    public void setDamping(float damping) {
        motor.damping = damping;
   }

    public float getRestitution() {
        return motor.restitution;
    }

    public void setRestitution(float restitution) {
        motor.restitution = restitution;
    }

	@Override
	public Class getClassTag() {
		// TODO Auto-generated method stub
		return this.getClass();
	}

	@Override
	public void read(JMEImporter i) throws IOException {
		if(motor==null){
			throw new NullPointerException("Motor is Null.");
		}
		InputCapsule capsule = i.getCapsule(this);
		setDamping(capsule.readFloat("damping", 0));
		setLimitSoftness(capsule.readFloat("limitSoftness", 0));
		setRestitution(capsule.readFloat("restituition", 0));
		setLowerLimit((Vector3f) capsule.readSavable("loweLimit", Vector3f.ZERO));
		setUpperLimit((Vector3f)capsule.readSavable("upperLimit", Vector3f.ZERO));
		setAccumulatedImpulse((Vector3f)capsule.readSavable("accumulatedImpule", Vector3f.ZERO));
		
	}

	@Override
	public void write(JMEExporter e) throws IOException {
		OutputCapsule capsule = e.getCapsule(this);
		capsule.write(motor.damping, "damping", 0);
		capsule.write(motor.limitSoftness, "limitSoftness", 0);
		capsule.write(motor.restitution, "restitution", 0);
		capsule.write(getLowerLimit(), "lowerLimit", Vector3f.ZERO);
		capsule.write(getUpperLimit(), "upperLimit", Vector3f.ZERO);
		capsule.write(getAccumulatedImpulse(), "accumulatedImpulse", Vector3f.ZERO);
	}

}
