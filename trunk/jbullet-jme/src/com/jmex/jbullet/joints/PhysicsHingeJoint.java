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
import com.jme.math.Vector3f;
import com.jmex.jbullet.PhysicsSpace;
import com.jmex.jbullet.nodes.PhysicsNode;
import com.jmex.jbullet.util.Converter;

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
    private float targetVelocity=0.0f;
    private float maxMotorImpule=0.0f;
    private float limitLow=1e30f;
    private float limitHigh=-1e30f;;
    private float softness=0.9f;
    private float biasFactor=0.3f;
    private float relaxationFactor=1.0f;

    private boolean enableMotor=false;
    private boolean applyMotor=false;
    private boolean applyLimit=false;

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
        this.enableMotor=enable;
        this.targetVelocity=targetVelocity;
        this.maxMotorImpule=maxMotorImpulse;
        applyMotor=true;
    }

	public void setLimit(float low, float high) {
        setLimit(low,high,0.9f,0.3f,1.0f);
    }

	public void setLimit(float low, float high, float _softness, float _biasFactor, float _relaxationFactor) {
        this.limitLow=low;
        this.limitHigh=high;
        this.softness=_softness;
        this.biasFactor=_biasFactor;
        this.relaxationFactor=_relaxationFactor;
        applyLimit=true;
    }

    @Override
    public void syncPhysics() {
        super.syncPhysics();
        if(applyLimit){
            ((HingeConstraint)constraint).setLimit(limitLow, limitHigh, softness, biasFactor, relaxationFactor);
            applyLimit=false;
        }
        if(applyMotor){
            ((HingeConstraint)constraint).enableAngularMotor(enableMotor, targetVelocity, maxMotorImpule);
            applyMotor=false;
        }
    }
    
}
