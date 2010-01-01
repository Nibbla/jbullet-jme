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

import com.bulletphysics.dynamics.constraintsolver.HingeConstraint;
import com.bulletphysics.dynamics.constraintsolver.SliderConstraint;
import com.bulletphysics.linearmath.Transform;
import com.jme.math.Matrix3f;
import com.jme.math.Vector3f;
import com.jmex.jbullet.nodes.PhysicsNode;
import com.jmex.jbullet.util.Converter;
import java.util.concurrent.Callable;

/**
 * <i>From bullet manual:</i><br>
 * The slider constraint allows the body to rotate around one axis and translate along this axis.
 * @author normenhansen
 */
public class PhysicsSliderJoint extends PhysicsJoint{
	public static final float SLIDER_CONSTRAINT_DEF_SOFTNESS    = 1.0f;
	public static final float SLIDER_CONSTRAINT_DEF_DAMPING     = 1.0f;
	public static final float SLIDER_CONSTRAINT_DEF_RESTITUTION = 0.7f;

	// use frameA fo define limits, if true
	private boolean useLinearReferenceFrameA;
	// linear limits
	private float lowerLinLimit;
	private float upperLinLimit;
	// angular limits
	private float lowerAngLimit;
	private float upperAngLimit;

	private float softnessDirLin;
	private float restitutionDirLin;
	private float dampingDirLin;
	private float softnessDirAng;
	private float restitutionDirAng;
	private float dampingDirAng;
	private float softnessLimLin;
	private float restitutionLimLin;
	private float dampingLimLin;
	private float softnessLimAng;
	private float restitutionLimAng;
	private float dampingLimAng;
	private float softnessOrthoLin;
	private float restitutionOrthoLin;
	private float dampingOrthoLin;
	private float softnessOrthoAng;
	private float restitutionOrthoAng;
	private float dampingOrthoAng;
    
	private boolean poweredLinMotor;
	private float targetLinMotorVelocity;
	private float maxLinMotorForce;
//	private float accumulatedLinMotorImpulse;

	private boolean poweredAngMotor;
	private float targetAngMotorVelocity;
	private float maxAngMotorForce;
//	private float accumulatedAngMotorImpulse;

    private Matrix3f rotA;
    private Matrix3f rotB;

    
    public PhysicsSliderJoint(PhysicsNode nodeA, PhysicsNode nodeB, Vector3f pivotA, Vector3f pivotB, Matrix3f rotA, Matrix3f rotB, boolean useLinearReferenceFrameA) {
        super(nodeA, nodeB, pivotA, pivotB);
        this.useLinearReferenceFrameA=useLinearReferenceFrameA;
        this.rotA=rotA;
        this.rotB=rotB;
        
        setDefaults();

        Transform transA=new Transform(Converter.convert(rotA));
        Converter.convert(pivotA,transA.origin);
        Converter.convert(rotA,transA.basis);

        Transform transB=new Transform(Converter.convert(rotB));
        Converter.convert(pivotB,transB.origin);
        Converter.convert(rotB,transB.basis);

        constraint=new SliderConstraint(nodeA.getRigidBody(), nodeB.getRigidBody(), transA, transB, useLinearReferenceFrameA);
        updateJoint();
    }

    public PhysicsSliderJoint(PhysicsNode nodeA, PhysicsNode nodeB, Vector3f pivotA, Vector3f pivotB, boolean useLinearReferenceFrameA) {
        super(nodeA, nodeB, pivotA, pivotB);
        this.useLinearReferenceFrameA=useLinearReferenceFrameA;
        this.rotA=new Matrix3f();
        this.rotB=new Matrix3f();

        setDefaults();

        Transform transA=new Transform(Converter.convert(rotA));
        Converter.convert(pivotA,transA.origin);

        Transform transB=new Transform(Converter.convert(rotB));
        Converter.convert(pivotB,transB.origin);

        constraint=new SliderConstraint(nodeA.getRigidBody(), nodeB.getRigidBody(), transA, transB, useLinearReferenceFrameA);
        updateJoint();
    }

    private void setDefaults() {
		lowerLinLimit = 1f;
		upperLinLimit = -1f;
		lowerAngLimit = 0f;
		upperAngLimit = 0f;
		softnessDirLin = SLIDER_CONSTRAINT_DEF_SOFTNESS;
		restitutionDirLin = SLIDER_CONSTRAINT_DEF_RESTITUTION;
		dampingDirLin = 0f;
		softnessDirAng = SLIDER_CONSTRAINT_DEF_SOFTNESS;
		restitutionDirAng = SLIDER_CONSTRAINT_DEF_RESTITUTION;
		dampingDirAng = 0f;
		softnessOrthoLin = SLIDER_CONSTRAINT_DEF_SOFTNESS;
		restitutionOrthoLin = SLIDER_CONSTRAINT_DEF_RESTITUTION;
		dampingOrthoLin = SLIDER_CONSTRAINT_DEF_DAMPING;
		softnessOrthoAng = SLIDER_CONSTRAINT_DEF_SOFTNESS;
		restitutionOrthoAng = SLIDER_CONSTRAINT_DEF_RESTITUTION;
		dampingOrthoAng = SLIDER_CONSTRAINT_DEF_DAMPING;
		softnessLimLin = SLIDER_CONSTRAINT_DEF_SOFTNESS;
		restitutionLimLin = SLIDER_CONSTRAINT_DEF_RESTITUTION;
		dampingLimLin = SLIDER_CONSTRAINT_DEF_DAMPING;
		softnessLimAng = SLIDER_CONSTRAINT_DEF_SOFTNESS;
		restitutionLimAng = SLIDER_CONSTRAINT_DEF_RESTITUTION;
		dampingLimAng = SLIDER_CONSTRAINT_DEF_DAMPING;

		poweredLinMotor = false;
		targetLinMotorVelocity = 0f;
		maxLinMotorForce = 0f;
//		accumulatedLinMotorImpulse = 0f;

		poweredAngMotor = false;
		targetAngMotorVelocity = 0f;
		maxAngMotorForce = 0f;
//		accumulatedAngMotorImpulse = 0f;
    }

    public float getLowerLinLimit() {
        return lowerLinLimit;
    }

    public void setLowerLinLimit(float lowerLinLimit) {
        this.lowerLinLimit = lowerLinLimit;
        pQueue.enqueue(doUpdateJoint);
    }

    public float getUpperLinLimit() {
        return upperLinLimit;
    }

    public void setUpperLinLimit(float upperLinLimit) {
        this.upperLinLimit = upperLinLimit;
        pQueue.enqueue(doUpdateJoint);
    }

    public float getLowerAngLimit() {
        return lowerAngLimit;
    }

    public void setLowerAngLimit(float lowerAngLimit) {
        this.lowerAngLimit = lowerAngLimit;
        pQueue.enqueue(doUpdateJoint);
    }

    public float getUpperAngLimit() {
        return upperAngLimit;
    }

    public void setUpperAngLimit(float upperAngLimit) {
        this.upperAngLimit = upperAngLimit;
        pQueue.enqueue(doUpdateJoint);
    }

    public float getSoftnessDirLin() {
        return softnessDirLin;
    }

    public void setSoftnessDirLin(float softnessDirLin) {
        this.softnessDirLin = softnessDirLin;
        pQueue.enqueue(doUpdateJoint);
    }

    public float getRestitutionDirLin() {
        return restitutionDirLin;
    }

    public void setRestitutionDirLin(float restitutionDirLin) {
        this.restitutionDirLin = restitutionDirLin;
        pQueue.enqueue(doUpdateJoint);
    }

    public float getDampingDirLin() {
        return dampingDirLin;
    }

    public void setDampingDirLin(float dampingDirLin) {
        this.dampingDirLin = dampingDirLin;
        pQueue.enqueue(doUpdateJoint);
    }

    public float getSoftnessDirAng() {
        return softnessDirAng;
    }

    public void setSoftnessDirAng(float softnessDirAng) {
        this.softnessDirAng = softnessDirAng;
        pQueue.enqueue(doUpdateJoint);
    }

    public float getRestitutionDirAng() {
        return restitutionDirAng;
    }

    public void setRestitutionDirAng(float restitutionDirAng) {
        this.restitutionDirAng = restitutionDirAng;
        pQueue.enqueue(doUpdateJoint);
    }

    public float getDampingDirAng() {
        return dampingDirAng;
    }

    public void setDampingDirAng(float dampingDirAng) {
        this.dampingDirAng = dampingDirAng;
        pQueue.enqueue(doUpdateJoint);
    }

    public float getSoftnessLimLin() {
        return softnessLimLin;
    }

    public void setSoftnessLimLin(float softnessLimLin) {
        this.softnessLimLin = softnessLimLin;
        pQueue.enqueue(doUpdateJoint);
    }

    public float getRestitutionLimLin() {
        return restitutionLimLin;
    }

    public void setRestitutionLimLin(float restitutionLimLin) {
        this.restitutionLimLin = restitutionLimLin;
        pQueue.enqueue(doUpdateJoint);
    }

    public float getDampingLimLin() {
        return dampingLimLin;
    }

    public void setDampingLimLin(float dampingLimLin) {
        this.dampingLimLin = dampingLimLin;
        pQueue.enqueue(doUpdateJoint);
    }

    public float getSoftnessLimAng() {
        return softnessLimAng;
    }

    public void setSoftnessLimAng(float softnessLimAng) {
        this.softnessLimAng = softnessLimAng;
        pQueue.enqueue(doUpdateJoint);
    }

    public float getRestitutionLimAng() {
        return restitutionLimAng;
    }

    public void setRestitutionLimAng(float restitutionLimAng) {
        this.restitutionLimAng = restitutionLimAng;
        pQueue.enqueue(doUpdateJoint);
    }

    public float getDampingLimAng() {
        return dampingLimAng;
    }

    public void setDampingLimAng(float dampingLimAng) {
        this.dampingLimAng = dampingLimAng;
        pQueue.enqueue(doUpdateJoint);
    }

    public float getSoftnessOrthoLin() {
        return softnessOrthoLin;
    }

    public void setSoftnessOrthoLin(float softnessOrthoLin) {
        this.softnessOrthoLin = softnessOrthoLin;
        pQueue.enqueue(doUpdateJoint);
    }

    public float getRestitutionOrthoLin() {
        return restitutionOrthoLin;
    }

    public void setRestitutionOrthoLin(float restitutionOrthoLin) {
        this.restitutionOrthoLin = restitutionOrthoLin;
        pQueue.enqueue(doUpdateJoint);
    }

    public float getDampingOrthoLin() {
        return dampingOrthoLin;
    }

    public void setDampingOrthoLin(float dampingOrthoLin) {
        this.dampingOrthoLin = dampingOrthoLin;
        pQueue.enqueue(doUpdateJoint);
    }

    public float getSoftnessOrthoAng() {
        return softnessOrthoAng;
    }

    public void setSoftnessOrthoAng(float softnessOrthoAng) {
        this.softnessOrthoAng = softnessOrthoAng;
        pQueue.enqueue(doUpdateJoint);
    }

    public float getRestitutionOrthoAng() {
        return restitutionOrthoAng;
    }

    public void setRestitutionOrthoAng(float restitutionOrthoAng) {
        this.restitutionOrthoAng = restitutionOrthoAng;
        pQueue.enqueue(doUpdateJoint);
    }

    public float getDampingOrthoAng() {
        return dampingOrthoAng;
    }

    public void setDampingOrthoAng(float dampingOrthoAng) {
        this.dampingOrthoAng = dampingOrthoAng;
        pQueue.enqueue(doUpdateJoint);
    }

    public boolean isPoweredLinMotor() {
        return poweredLinMotor;
    }

    public void setPoweredLinMotor(boolean poweredLinMotor) {
        this.poweredLinMotor = poweredLinMotor;
        pQueue.enqueue(doUpdateMotor);
    }

    public float getTargetLinMotorVelocity() {
        return targetLinMotorVelocity;
    }

    public void setTargetLinMotorVelocity(float targetLinMotorVelocity) {
        this.targetLinMotorVelocity = targetLinMotorVelocity;
        pQueue.enqueue(doUpdateMotor);
    }

    public float getMaxLinMotorForce() {
        return maxLinMotorForce;
    }

    public void setMaxLinMotorForce(float maxLinMotorForce) {
        this.maxLinMotorForce = maxLinMotorForce;
        pQueue.enqueue(doUpdateMotor);
    }

    public boolean isPoweredAngMotor() {
        return poweredAngMotor;
    }

    public void setPoweredAngMotor(boolean poweredAngMotor) {
        this.poweredAngMotor = poweredAngMotor;
        pQueue.enqueue(doUpdateMotor);
    }

    public float getTargetAngMotorVelocity() {
        return targetAngMotorVelocity;
    }

    public void setTargetAngMotorVelocity(float targetAngMotorVelocity) {
        this.targetAngMotorVelocity = targetAngMotorVelocity;
        pQueue.enqueue(doUpdateMotor);
    }

    public float getMaxAngMotorForce() {
        return maxAngMotorForce;
    }

    public void setMaxAngMotorForce(float maxAngMotorForce) {
        this.maxAngMotorForce = maxAngMotorForce;
        pQueue.enqueue(doUpdateMotor);
    }

    private void updateJoint() {
        ((SliderConstraint)constraint).setLowerAngLimit(lowerAngLimit);
        ((SliderConstraint)constraint).setLowerLinLimit(lowerLinLimit);
        ((SliderConstraint)constraint).setUpperAngLimit(upperAngLimit);
        ((SliderConstraint)constraint).setUpperLinLimit(upperLinLimit);

        ((SliderConstraint)constraint).setDampingDirAng(dampingDirAng);
        ((SliderConstraint)constraint).setDampingDirLin(dampingDirLin);
        ((SliderConstraint)constraint).setDampingLimAng(dampingLimAng);
        ((SliderConstraint)constraint).setDampingLimLin(dampingLimLin);
        ((SliderConstraint)constraint).setDampingOrthoAng(dampingOrthoAng);
        ((SliderConstraint)constraint).setDampingOrthoLin(dampingOrthoLin);

        ((SliderConstraint)constraint).setRestitutionDirAng(restitutionDirAng);
        ((SliderConstraint)constraint).setRestitutionDirLin(restitutionDirLin);
        ((SliderConstraint)constraint).setRestitutionLimAng(restitutionLimAng);
        ((SliderConstraint)constraint).setRestitutionLimLin(restitutionLimLin);
        ((SliderConstraint)constraint).setRestitutionOrthoAng(restitutionOrthoAng);
        ((SliderConstraint)constraint).setRestitutionOrthoLin(restitutionOrthoLin);
        ((SliderConstraint)constraint).setSoftnessDirAng(softnessDirAng);
        ((SliderConstraint)constraint).setSoftnessDirLin(softnessDirLin);
        ((SliderConstraint)constraint).setSoftnessLimAng(softnessLimAng);
        ((SliderConstraint)constraint).setSoftnessLimLin(softnessLimLin);
        ((SliderConstraint)constraint).setSoftnessOrthoAng(softnessOrthoAng);
        ((SliderConstraint)constraint).setSoftnessOrthoLin(softnessOrthoLin);

    }

    private Callable doUpdateJoint=new Callable(){
        public Object call() throws Exception {
            updateJoint();
            return null;
        }
    };

    private void updateMotors(){
        ((SliderConstraint)constraint).setMaxAngMotorForce(maxAngMotorForce);
        ((SliderConstraint)constraint).setMaxLinMotorForce(maxLinMotorForce);

        ((SliderConstraint)constraint).setTargetAngMotorVelocity(targetAngMotorVelocity);
        ((SliderConstraint)constraint).setTargetLinMotorVelocity(targetLinMotorVelocity);

        ((SliderConstraint)constraint).setPoweredAngMotor(collisionBetweenLinkedBodys);
        ((SliderConstraint)constraint).setPoweredLinMotor(collisionBetweenLinkedBodys);
    }

    private Callable doUpdateMotor=new Callable(){
        public Object call() throws Exception {
            updateMotors();
            return null;
        }
    };

}