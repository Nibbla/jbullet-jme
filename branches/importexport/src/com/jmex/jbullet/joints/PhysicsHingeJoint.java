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
package com.jmex.jbullet.joints;

import java.io.IOException;

import com.bulletphysics.dynamics.constraintsolver.HingeConstraint;
import com.jme.math.Vector3f;
import com.jme.util.export.InputCapsule;
import com.jme.util.export.JMEExporter;
import com.jme.util.export.JMEImporter;
import com.jme.util.export.OutputCapsule;
import com.jmex.jbullet.PhysicsSpace;
import com.jmex.jbullet.nodes.PhysicsNode;
import com.jmex.jbullet.util.Converter;
import com.jmex.model.collada.schema.capsuleType;

/**
 * <i>From bullet manual:</i><br>
 * Hinge constraint, or revolute joint restricts two additional angular degrees of freedom,
 * so the body can only rotate around one axis, the hinge axis.
 * This can be useful to represent doors or wheels rotating around one axis.
 * The user can specify limits and motor for the hinge.
 * @author normenhansen
 */
public class PhysicsHingeJoint extends PhysicsJoint{
    protected Vector3f axisA;
    protected Vector3f axisB;
    
    
    private float limitSoftness=0.9f; 
	private float biasFactor=0.3f; 
	private float relaxationFactor=1; 
	private float lowerLimit=1e30f;;	
	private float upperLimit=-1e30f;;
    
	public PhysicsHingeJoint(){
	}
    /**
     * Creates a new HingeJoint
     * @param pivotA local translation of the joint connection point in node A
     * @param pivotB local translation of the joint connection point in node A
     */
    public PhysicsHingeJoint(PhysicsNode nodeA, PhysicsNode nodeB, Vector3f pivotA, Vector3f pivotB, Vector3f axisA, Vector3f axisB) {
        super(nodeA, nodeB, pivotA, pivotB);
        this.axisA=axisA;
        this.axisB=axisB;
        constraint=new HingeConstraint(nodeA.getRigidBody(), nodeB.getRigidBody(),
                Converter.convert(pivotA), Converter.convert(pivotB),
                Converter.convert(axisA), Converter.convert(axisB));
    }

    public void enableMotor(boolean enable, float targetVelocity, float maxMotorImpulse){
        ((HingeConstraint)constraint).enableAngularMotor(enable, targetVelocity, maxMotorImpulse);
    }

	public void setLimit(float low, float high) {
		this.lowerLimit=low;
		this.upperLimit=high;
        ((HingeConstraint)constraint).setLimit(low,high);
    }

	public void setLimit(float low, float high, float _softness, float _biasFactor, float _relaxationFactor) {
		this.lowerLimit=low;
		this.upperLimit=high;
		this.limitSoftness=_softness;
		this.biasFactor=_biasFactor;
		this.relaxationFactor=_relaxationFactor;
		((HingeConstraint)constraint).setLimit(low, high, _softness, _biasFactor, _relaxationFactor);
    }
	
	@Override
	public Class getClassTag() {
		return this.getClass();
	}

	@Override
	public void read(JMEImporter im) throws IOException {
		super.read(im);
		InputCapsule capsule= im.getCapsule(this);
		axisA = (Vector3f) capsule.readSavable("axisA", null);
		axisB = (Vector3f) capsule.readSavable("axisB", null);
		lowerLimit = capsule.readFloat("lowerLimit", 0);
		upperLimit= capsule.readFloat("upperLimit", 0);
		limitSoftness= capsule.readFloat("limitSoftness", 0);
		biasFactor= capsule.readFloat("biasFactor", 0);
		relaxationFactor= capsule.readFloat("relaxationFactor", 0);
		constraint =new HingeConstraint(nodeA.getRigidBody(), nodeB.getRigidBody(),
                Converter.convert(pivotA), Converter.convert(pivotB),
                Converter.convert(axisA), Converter.convert(axisB));
		((HingeConstraint)constraint).setLimit(lowerLimit, upperLimit, limitSoftness, biasFactor, relaxationFactor);
		
		boolean enableMotor = capsule.readBoolean("enableMotor", false);
		float vel =capsule.readFloat("velocity", 0);
		float impl = capsule.readFloat("maximpulse", 0);
		enableMotor(enableMotor, vel, impl);
		
		
		//moved to PhysicNode
		//PhysicsSpace.getPhysicsSpace().add(this);
		
	}

	@Override
	public void write(JMEExporter ex) throws IOException {
		super.write(ex);
		OutputCapsule capsule = ex.getCapsule(this);
		capsule.write(axisA, "axisA", null);
		capsule.write(axisB, "axisB", null);
		
		capsule.write(lowerLimit, "lowerLimit", 0);
		capsule.write(upperLimit, "upperLimit", 0);
		capsule.write(limitSoftness, "limitSoftness", 0);
		capsule.write(biasFactor, "biasFactor", 0);
		capsule.write(relaxationFactor, "relaxationFactor", 0);
		HingeConstraint hinge = (HingeConstraint)constraint;
		capsule.write(hinge.getEnableAngularMotor(), "enableMotor", false);
		capsule.write(hinge.getMotorTargetVelosity(), "velocity", 0);
		capsule.write(hinge.getMaxMotorImpulse(), "maximpulse", 0);
		
	}

}
