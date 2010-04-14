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

import com.bulletphysics.dynamics.constraintsolver.Generic6DofConstraint;
import com.bulletphysics.linearmath.Transform;
import com.jme.math.Matrix3f;
import com.jme.math.Vector3f;
import com.jme.util.export.InputCapsule;
import com.jme.util.export.JMEExporter;
import com.jme.util.export.JMEImporter;
import com.jme.util.export.OutputCapsule;
import com.jme.util.export.binary.BinaryClassLoader;
import com.jme.util.export.binary.BinaryLoaderModule;
import com.jmex.jbullet.PhysicsSpace;
import com.jmex.jbullet.binarymodules.RotationalLimitMotorModule;
import com.jmex.jbullet.binarymodules.TranslationalLimitMotorModule;
import com.jmex.jbullet.joints.motors.RotationalLimitMotor;
import com.jmex.jbullet.joints.motors.TranslationalLimitMotor;
import com.jmex.jbullet.nodes.PhysicsNode;
import com.jmex.jbullet.util.Converter;

import java.io.IOException;
import java.util.LinkedList;

import org.lwjgl.Sys;

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
    private LinkedList<RotationalLimitMotor> rotationalMotors=new LinkedList<RotationalLimitMotor>();
    private TranslationalLimitMotor translationalMotor;

    
    public Physics6DofJoint(){
    	
    }
    public Physics6DofJoint(PhysicsNode nodeA, PhysicsNode nodeB, Vector3f pivotA, Vector3f pivotB, Matrix3f rotA, Matrix3f rotB, boolean useLinearReferenceFrameA) {
        super(nodeA, nodeB, pivotA, pivotB);

        Transform transA=new Transform(Converter.convert(rotA));
        Converter.convert(pivotA,transA.origin);
        Converter.convert(rotA,transA.basis);

        Transform transB=new Transform(Converter.convert(rotB));
        Converter.convert(pivotB,transB.origin);
        Converter.convert(rotB,transB.basis);

        constraint=new Generic6DofConstraint(nodeA.getRigidBody(), nodeB.getRigidBody(), transA, transB, useLinearReferenceFrameA);
        gatherMotors();
    }

    public Physics6DofJoint(PhysicsNode nodeA, PhysicsNode nodeB, Vector3f pivotA, Vector3f pivotB, boolean useLinearReferenceFrameA) {
        super(nodeA, nodeB, pivotA, pivotB);

        Transform transA=new Transform(Converter.convert(new Matrix3f()));
        Converter.convert(pivotA,transA.origin);

        Transform transB=new Transform(Converter.convert(new Matrix3f()));
        Converter.convert(pivotB,transB.origin);

        constraint=new Generic6DofConstraint(nodeA.getRigidBody(), nodeB.getRigidBody(), transA, transB, useLinearReferenceFrameA);
        gatherMotors();
    }

    private void gatherMotors(){
        for (int i = 0; i < 3; i++) {
            RotationalLimitMotor rmot=new RotationalLimitMotor(((Generic6DofConstraint)constraint).getRotationalLimitMotor(i));
            rotationalMotors.add(rmot);
        }
        translationalMotor=new TranslationalLimitMotor(((Generic6DofConstraint)constraint).getTranslationalLimitMotor());
    }

    /**
     * returns the TranslationalLimitMotor of this 6DofJoint which allows
     * manipulating the translational axis
     * @return the TranslationalLimitMotor
     */
    public TranslationalLimitMotor getTranslationalLimitMotor() {
        return translationalMotor;
    }

    /**
     * returns one of the three RotationalLimitMotors of this 6DofJoint which
     * allow manipulating the rotational axes
     * @param index the index of the RotationalLimitMotor
     * @return the RotationalLimitMotor at the given index
     */
    public RotationalLimitMotor getRotationalLimitMotor(int index){
        return rotationalMotors.get(index);
    }

    public void setLinearUpperLimit(Vector3f vector){
        ((Generic6DofConstraint)constraint).setLinearUpperLimit(Converter.convert(vector));
    }

    public void setLinearLowerLimit(Vector3f vector){
        ((Generic6DofConstraint)constraint).setLinearLowerLimit(Converter.convert(vector));
    }

    public void setAngularUpperLimit(Vector3f vector){
        ((Generic6DofConstraint)constraint).setAngularUpperLimit(Converter.convert(vector));
    }

    public void setAngularLowerLimit(Vector3f vector){
        ((Generic6DofConstraint)constraint).setAngularLowerLimit(Converter.convert(vector));
    }
    
    @Override
	public Class getClassTag() {
		return this.getClass();
	}

	@Override
	public void read(JMEImporter im) throws IOException {
		super.read(im);
		//throw (new UnsupportedOperationException("Not implemented yet."));
		InputCapsule capsule = im.getCapsule(this);
		
		Matrix3f rotA = (Matrix3f) capsule.readSavable("rotA", null);
		Matrix3f rotB = (Matrix3f) capsule.readSavable("rotB", null);
		
		Transform transA=new Transform(Converter.convert(rotA));
        Converter.convert(pivotA,transA.origin);
        if(rotA!=null){
        	 Converter.convert(rotA,transA.basis);
        }

        Transform transB=new Transform(Converter.convert(rotB));
        Converter.convert(pivotB,transB.origin);
        Converter.convert(rotB,transB.basis);
        if(rotB!=null){
       	 Converter.convert(rotB,transB.basis);
       }
        constraint=new Generic6DofConstraint(nodeA.getRigidBody(), nodeB.getRigidBody(), transA, transB, false);
        
        for (int i = 0; i < 3; i++) {
        	RotationalLimitMotorModule module = new RotationalLimitMotorModule(((Generic6DofConstraint)constraint).getRotationalLimitMotor(i));
        	BinaryClassLoader.registerModule(module);
        	RotationalLimitMotor rmot= (RotationalLimitMotor) capsule.readSavable("rotationalMotor"+i, null);
            rotationalMotors.add(rmot);
            BinaryClassLoader.unregisterModule(module);
        }
        
        TranslationalLimitMotorModule module =new TranslationalLimitMotorModule(((Generic6DofConstraint)constraint).getTranslationalLimitMotor());
        BinaryClassLoader.registerModule(module);
        translationalMotor=(TranslationalLimitMotor) capsule.readSavable("translationalMotor", null);
        BinaryClassLoader.unregisterModule(module);
        PhysicsSpace.getPhysicsSpace().add(this);
	}

	@Override
	public void write(JMEExporter ex) throws IOException {
		super.write(ex);
		//throw (new UnsupportedOperationException("Not implemented yet."));
		OutputCapsule capsule = ex.getCapsule(this);
		Transform trans = new Transform();
		//TODO rotA & rotB are getCalculatedTransform?
		((Generic6DofConstraint)constraint).getFrameOffsetA(trans);
		capsule.write(Converter.convert(trans.basis), "rotA", null);
		((Generic6DofConstraint)constraint).getFrameOffsetB(trans);
		capsule.write(Converter.convert(trans.basis), "rotB", null);
		capsule.write(translationalMotor, "translationalMotor", null);
		capsule.write(getRotationalLimitMotor(0), "rotationalMotor0", null);
		capsule.write(getRotationalLimitMotor(1), "rotationalMotor1", null);
		capsule.write(getRotationalLimitMotor(2), "rotationalMotor2", null);
		//TODO useLinearReferenceFrameA
	}
}
