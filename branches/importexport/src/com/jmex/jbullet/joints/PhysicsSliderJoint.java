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

import com.bulletphysics.dynamics.constraintsolver.SliderConstraint;
import com.bulletphysics.linearmath.Transform;
import com.jme.math.Matrix3f;
import com.jme.math.Vector3f;
import com.jme.util.export.JMEExporter;
import com.jme.util.export.JMEImporter;
import com.jme.util.export.OutputCapsule;
import com.jmex.jbullet.nodes.PhysicsNode;
import com.jmex.jbullet.util.Converter;

/**
 * <i>From bullet manual:</i><br>
 * The slider constraint allows the body to rotate around one axis and translate along this axis.
 * @author normenhansen
 */
public class PhysicsSliderJoint extends PhysicsJoint{
    
    public PhysicsSliderJoint(PhysicsNode nodeA, PhysicsNode nodeB, Vector3f pivotA, Vector3f pivotB, Matrix3f rotA, Matrix3f rotB, boolean useLinearReferenceFrameA) {
        super(nodeA, nodeB, pivotA, pivotB);

        Transform transA=new Transform(Converter.convert(rotA));
        Converter.convert(pivotA,transA.origin);
        Converter.convert(rotA,transA.basis);

        Transform transB=new Transform(Converter.convert(rotB));
        Converter.convert(pivotB,transB.origin);
        Converter.convert(rotB,transB.basis);

        constraint=new SliderConstraint(nodeA.getRigidBody(), nodeB.getRigidBody(), transA, transB, useLinearReferenceFrameA);
    }

    public PhysicsSliderJoint(PhysicsNode nodeA, PhysicsNode nodeB, Vector3f pivotA, Vector3f pivotB, boolean useLinearReferenceFrameA) {
        super(nodeA, nodeB, pivotA, pivotB);

        Transform transA=new Transform(Converter.convert(new Matrix3f()));
        Converter.convert(pivotA,transA.origin);

        Transform transB=new Transform(Converter.convert(new Matrix3f()));
        Converter.convert(pivotB,transB.origin);

        constraint=new SliderConstraint(nodeA.getRigidBody(), nodeB.getRigidBody(), transA, transB, useLinearReferenceFrameA);
    }

    public float getLowerLinLimit() {
        return ((SliderConstraint)constraint).getLowerLinLimit();
    }

    public void setLowerLinLimit(float lowerLinLimit) {
        ((SliderConstraint)constraint).setLowerLinLimit(lowerLinLimit);
    }

    public float getUpperLinLimit() {
        return ((SliderConstraint)constraint).getUpperLinLimit();
    }

    public void setUpperLinLimit(float upperLinLimit) {
        ((SliderConstraint)constraint).setUpperLinLimit(upperLinLimit);
    }

    public float getLowerAngLimit() {
        return ((SliderConstraint)constraint).getLowerAngLimit();
    }

    public void setLowerAngLimit(float lowerAngLimit) {
        ((SliderConstraint)constraint).setLowerAngLimit(lowerAngLimit);
    }

    public float getUpperAngLimit() {
        return ((SliderConstraint)constraint).getUpperAngLimit();
    }

    public void setUpperAngLimit(float upperAngLimit) {
        ((SliderConstraint)constraint).setUpperAngLimit(upperAngLimit);
    }

    public float getSoftnessDirLin() {
        return ((SliderConstraint)constraint).getSoftnessDirLin();
    }

    public void setSoftnessDirLin(float softnessDirLin) {
        ((SliderConstraint)constraint).setSoftnessDirLin(softnessDirLin);
    }

    public float getRestitutionDirLin() {
        return ((SliderConstraint)constraint).getRestitutionDirLin();
    }

    public void setRestitutionDirLin(float restitutionDirLin) {
        ((SliderConstraint)constraint).setRestitutionDirLin(restitutionDirLin);
    }

    public float getDampingDirLin() {
        return ((SliderConstraint)constraint).getDampingDirLin();
    }

    public void setDampingDirLin(float dampingDirLin) {
        ((SliderConstraint)constraint).setDampingDirLin(dampingDirLin);
    }

    public float getSoftnessDirAng() {
        return ((SliderConstraint)constraint).getSoftnessDirAng();
    }

    public void setSoftnessDirAng(float softnessDirAng) {
        ((SliderConstraint)constraint).setSoftnessDirAng(softnessDirAng);
    }

    public float getRestitutionDirAng() {
        return ((SliderConstraint)constraint).getRestitutionDirAng();
    }

    public void setRestitutionDirAng(float restitutionDirAng) {
        ((SliderConstraint)constraint).setRestitutionDirAng(restitutionDirAng);
    }

    public float getDampingDirAng() {
        return ((SliderConstraint)constraint).getDampingDirAng();
    }

    public void setDampingDirAng(float dampingDirAng) {
        ((SliderConstraint)constraint).setDampingDirAng(dampingDirAng);
    }

    public float getSoftnessLimLin() {
        return ((SliderConstraint)constraint).getSoftnessLimLin();
    }

    public void setSoftnessLimLin(float softnessLimLin) {
        ((SliderConstraint)constraint).setSoftnessLimLin(softnessLimLin);
    }

    public float getRestitutionLimLin() {
        return ((SliderConstraint)constraint).getRestitutionLimLin();
    }

    public void setRestitutionLimLin(float restitutionLimLin) {
        ((SliderConstraint)constraint).setRestitutionLimLin(restitutionLimLin);
    }

    public float getDampingLimLin() {
        return ((SliderConstraint)constraint).getDampingLimLin();
    }

    public void setDampingLimLin(float dampingLimLin) {
        ((SliderConstraint)constraint).setDampingLimLin(dampingLimLin);
    }

    public float getSoftnessLimAng() {
        return ((SliderConstraint)constraint).getSoftnessLimAng();
    }

    public void setSoftnessLimAng(float softnessLimAng) {
        ((SliderConstraint)constraint).setSoftnessLimAng(softnessLimAng);
    }

    public float getRestitutionLimAng() {
        return ((SliderConstraint)constraint).getRestitutionLimAng();
    }

    public void setRestitutionLimAng(float restitutionLimAng) {
        ((SliderConstraint)constraint).setRestitutionLimAng (restitutionLimAng);
    }

    public float getDampingLimAng() {
        return ((SliderConstraint)constraint).getDampingLimAng();
    }

    public void setDampingLimAng(float dampingLimAng) {
        ((SliderConstraint)constraint).setDampingLimAng(dampingLimAng);
    }

    public float getSoftnessOrthoLin() {
        return ((SliderConstraint)constraint).getSoftnessOrthoLin();
    }

    public void setSoftnessOrthoLin(float softnessOrthoLin) {
        ((SliderConstraint)constraint).setSoftnessOrthoLin(softnessOrthoLin);
    }

    public float getRestitutionOrthoLin() {
        return ((SliderConstraint)constraint).getRestitutionOrthoLin();
    }

    public void setRestitutionOrthoLin(float restitutionOrthoLin) {
        ((SliderConstraint)constraint).setRestitutionOrthoLin(restitutionOrthoLin);
    }

    public float getDampingOrthoLin() {
        return ((SliderConstraint)constraint).getDampingOrthoLin();
    }

    public void setDampingOrthoLin(float dampingOrthoLin) {
        ((SliderConstraint)constraint).setDampingOrthoLin(dampingOrthoLin);
    }

    public float getSoftnessOrthoAng() {
        return ((SliderConstraint)constraint).getSoftnessOrthoAng();
    }

    public void setSoftnessOrthoAng(float softnessOrthoAng) {
        ((SliderConstraint)constraint).setSoftnessOrthoAng(softnessOrthoAng);
    }

    public float getRestitutionOrthoAng() {
        return ((SliderConstraint)constraint).getRestitutionOrthoAng();
    }

    public void setRestitutionOrthoAng(float restitutionOrthoAng) {
        ((SliderConstraint)constraint).setRestitutionOrthoAng(restitutionOrthoAng);
    }

    public float getDampingOrthoAng() {
        return ((SliderConstraint)constraint).getDampingOrthoAng();
    }

    public void setDampingOrthoAng(float dampingOrthoAng) {
        ((SliderConstraint)constraint).setDampingOrthoAng(dampingOrthoAng);
    }

    public boolean isPoweredLinMotor() {
        return ((SliderConstraint)constraint).getPoweredLinMotor();
    }

    public void setPoweredLinMotor(boolean poweredLinMotor) {
        ((SliderConstraint)constraint).setPoweredLinMotor(poweredLinMotor);
    }

    public float getTargetLinMotorVelocity() {
        return ((SliderConstraint)constraint).getTargetLinMotorVelocity();
    }

    public void setTargetLinMotorVelocity(float targetLinMotorVelocity) {
        ((SliderConstraint)constraint).setTargetLinMotorVelocity(targetLinMotorVelocity);
    }

    public float getMaxLinMotorForce() {
        return ((SliderConstraint)constraint).getMaxLinMotorForce();
    }

    public void setMaxLinMotorForce(float maxLinMotorForce) {
        ((SliderConstraint)constraint).setMaxLinMotorForce(maxLinMotorForce);
    }

    public boolean isPoweredAngMotor() {
        return ((SliderConstraint)constraint).getPoweredAngMotor();
    }

    public void setPoweredAngMotor(boolean poweredAngMotor) {
        ((SliderConstraint)constraint).setPoweredAngMotor(poweredAngMotor);
    }

    public float getTargetAngMotorVelocity() {
        return ((SliderConstraint)constraint).getTargetAngMotorVelocity();
    }

    public void setTargetAngMotorVelocity(float targetAngMotorVelocity) {
        ((SliderConstraint)constraint).setTargetAngMotorVelocity(targetAngMotorVelocity);
    }

    public float getMaxAngMotorForce() {
        return ((SliderConstraint)constraint).getMaxAngMotorForce();
    }

    public void setMaxAngMotorForce(float maxAngMotorForce) {
        ((SliderConstraint)constraint).setMaxAngMotorForce(maxAngMotorForce);
    }
    
    @Override
	public Class getClassTag() {
		return this.getClass();
	}

	@Override
	public void read(JMEImporter im) throws IOException {
		throw (new UnsupportedOperationException("Not implemented yet."));
	}

	@Override
	public void write(JMEExporter ex) throws IOException {
		super.write(ex);
		OutputCapsule capsule = ex.getCapsule(this);
		SliderConstraint slider = (SliderConstraint)constraint;
		capsule.write(slider.getLowerLinLimit(), "LowerLinLimit", 0);
		capsule.write(slider.getUpperLinLimit(), "UpperLinLimit", 0);
		capsule.write(slider.getLowerAngLimit(), "LowerAngLimit", 0);
		capsule.write(slider.getUpperAngLimit(), "UpperAngLimit", 0);
		capsule.write(slider.getSoftnessDirAng(), "SoftnessDirAng", 0);
		capsule.write(slider.getSoftnessDirLin(), "SoftnessDirLin", 0);
		capsule.write(slider.getRestitutionDirLin(), "RestitutionDirLin", 0);
		capsule.write(slider.getDampingDirLin(), "DampingDirLin", 0);
		capsule.write(slider.getRestitutionDirAng(), "RestitutionDirAng", 0);
		capsule.write(slider.getDampingDirAng(), "DampingDirAng", 0);
		capsule.write(slider.getLowerLinLimit(), "LowerLinLimit", 0);
		capsule.write(slider.getLowerLinLimit(), "LowerLinLimit", 0);
		capsule.write(slider.getLowerLinLimit(), "LowerLinLimit", 0);
		capsule.write(slider.getLowerLinLimit(), "LowerLinLimit", 0);
		//...
		throw (new UnsupportedOperationException("Not implemented yet."));
	}

}
