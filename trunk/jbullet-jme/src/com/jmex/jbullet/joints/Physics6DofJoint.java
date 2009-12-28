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

import com.bulletphysics.dynamics.constraintsolver.Generic6DofConstraint;
import com.bulletphysics.linearmath.Transform;
import com.jme.math.Matrix3f;
import com.jme.math.Vector3f;
import com.jmex.jbullet.PhysicsSpace;
import com.jmex.jbullet.node.PhysicsNode;
import com.jmex.jbullet.util.Converter;

/**
 * <i>From bullet manual:</i><br>
 * This generic constraint can emulate a variety of standard constraints,
 * by configuring each of the 6 degrees of freedom (dof).
 * The first 3 dof axis are linear axis, which represent translation of rigidbodies,
 * and the latter 3 dof axis represent the angular motion. Each axis can be either locked,
 * free or limited. On construction of a new btGeneric6DofConstraint, all axis are locked.
 * Afterwards the axis can be reconfigured. Note that several combinations that
 * include free and/or limited angular degrees of freedom are undefined.
 * @author normenhansen
 */
public class Physics6DofJoint extends PhysicsJoint{
    private Matrix3f rotA;
    private Matrix3f rotB;
	// use frameA fo define limits, if true
	private boolean useLinearReferenceFrameA;

    private Vector3f linearUpperLimit=new Vector3f();
    private Vector3f linearLowerLimit=new Vector3f();
    private Vector3f angularUpperLimit=new Vector3f();
    private Vector3f angularLowerLimit=new Vector3f();

    private boolean updateLimits=false;

    public Physics6DofJoint(PhysicsNode nodeA, PhysicsNode nodeB, Vector3f pivotA, Vector3f pivotB, Matrix3f rotA, Matrix3f rotB, boolean useLinearReferenceFrameA) {
        super(nodeA, nodeB, pivotA, pivotB);
        this.useLinearReferenceFrameA=useLinearReferenceFrameA;
        this.rotA=rotA;
        this.rotB=rotB;

//        setDefaults();

        Transform transA=new Transform(Converter.convert(rotA));
        Converter.convert(pivotA,transA.origin);
        Converter.convert(rotA,transA.basis);

        Transform transB=new Transform(Converter.convert(rotB));
        Converter.convert(pivotB,transB.origin);
        Converter.convert(rotB,transB.basis);

        constraint=new Generic6DofConstraint(nodeA.getRigidBody(), nodeB.getRigidBody(), transA, transB, useLinearReferenceFrameA);
//        updateJoint();
//        ((Generic6DofConstraint)constraint).
    }

    public Physics6DofJoint(PhysicsNode nodeA, PhysicsNode nodeB, Vector3f pivotA, Vector3f pivotB, boolean useLinearReferenceFrameA) {
        super(nodeA, nodeB, pivotA, pivotB);
        this.useLinearReferenceFrameA=useLinearReferenceFrameA;
        this.rotA=new Matrix3f();
        this.rotB=new Matrix3f();

//        setDefaults();

        Transform transA=new Transform(Converter.convert(rotA));
        Converter.convert(pivotA,transA.origin);

        Transform transB=new Transform(Converter.convert(rotB));
        Converter.convert(pivotB,transB.origin);

        constraint=new Generic6DofConstraint(nodeA.getRigidBody(), nodeB.getRigidBody(), transA, transB, useLinearReferenceFrameA);
//        updateJoint();
    }

    public void setLinearUpperLimit(Vector3f vector){
        this.linearUpperLimit.set(vector);
        updateLimits=true;
    }

    public void setLinearLowerLimit(Vector3f vector){
        this.linearUpperLimit.set(vector);
        updateLimits=true;
    }

    public void setAngularUpperLimit(Vector3f vector){
        this.angularUpperLimit.set(vector);
        updateLimits=true;
    }

    public void setAngularLowerLimit(Vector3f vector){
        this.angularUpperLimit.set(vector);
        updateLimits=true;
    }
    
    private void updateLimits(){
        ((Generic6DofConstraint)constraint).setLinearUpperLimit(Converter.convert(linearUpperLimit));
        ((Generic6DofConstraint)constraint).setLinearLowerLimit(Converter.convert(linearLowerLimit));
        ((Generic6DofConstraint)constraint).setAngularLowerLimit(Converter.convert(angularUpperLimit));
        ((Generic6DofConstraint)constraint).setAngularUpperLimit(Converter.convert(angularLowerLimit));
    }

    @Override
    public void syncPhysics() {
        if(updateLimits){
            updateLimits();
            updateLimits=false;
        }
        super.syncPhysics();
    }

}
