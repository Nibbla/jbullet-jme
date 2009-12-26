/*
 * Copyright (c) 2003-2009 jMonkeyEngine
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
 * * Neither the name of 'jMonkeyEngine' nor the names of its contributors
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

import com.bulletphysics.dynamics.constraintsolver.ConeTwistConstraint;
import com.bulletphysics.linearmath.Transform;
import com.jme.math.Matrix3f;
import com.jme.math.Vector3f;
import com.jmex.jbullet.PhysicsSpace;
import com.jmex.jbullet.node.PhysicsNode;
import com.jmex.jbullet.util.Converter;

/**
 * <i>From bullet manual:</i><br>
 * To create ragdolls, the conve twist constraint is very useful for limbs like the upper arm.
 * It is a special point to point constraint that adds cone and twist axis limits.
 * The x-axis serves as twist axis.
 * @author normenhansen
 */
public class PhysicsConeJoint extends PhysicsJoint{
    private Matrix3f rotA, rotB;
    private float swingSpan1=1e30f, swingSpan2=1e30f, twistSpan=1e30f;
    //TODO: softness
    private float softness, biasFactor=0.3f, relaxationFactor=1.0f;

    private boolean applyLimit=false;

    public PhysicsConeJoint(PhysicsNode nodeA, PhysicsNode nodeB, Vector3f pivotA, Vector3f pivotB) {
        super(nodeA, nodeB, pivotA, pivotB);
        this.rotA=new Matrix3f();
        this.rotB=new Matrix3f();
        
        Transform transA=new Transform();
        Converter.convert(pivotA,transA.origin);
        Transform transB=new Transform();
        Converter.convert(pivotB,transB.origin);
        constraint=new ConeTwistConstraint(nodeA.getRigidBody(), nodeB.getRigidBody(), transA, transB);
        PhysicsSpace.getPhysicsSpace().addJoint(this);
    }

    public PhysicsConeJoint(PhysicsNode nodeA, PhysicsNode nodeB, Vector3f pivotA, Vector3f pivotB, Matrix3f rotA, Matrix3f rotB) {
        super(nodeA, nodeB, pivotA, pivotB);
        this.rotA=rotA;
        this.rotB=rotB;

        Transform transA=new Transform(Converter.convert(rotA));
        Converter.convert(pivotA,transA.origin);
        Converter.convert(rotA,transA.basis);

        Transform transB=new Transform(Converter.convert(rotB));
        Converter.convert(pivotB,transB.origin);
        Converter.convert(rotB,transB.basis);
        
        constraint=new ConeTwistConstraint(nodeA.getRigidBody(), nodeB.getRigidBody(), transA, transB);
        PhysicsSpace.getPhysicsSpace().addJoint(this);
    }

    public void setLimit(float swingSpan1, float swingSpan2, float twistSpan) {
        this.swingSpan1=swingSpan1;
        this.swingSpan2=swingSpan2;
        this.twistSpan=twistSpan;
        applyLimit=true;
    }

    @Override
    public void syncPhysics() {
        if(applyLimit){
            ((ConeTwistConstraint)constraint).setLimit(swingSpan1, swingSpan2, twistSpan);
            applyLimit=false;
        }
        super.syncPhysics();
    }

}
