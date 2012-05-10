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

import com.bulletphysics.dynamics.constraintsolver.TypedConstraint;
import com.jme.math.Vector3f;
import com.jme.util.export.InputCapsule;
import com.jme.util.export.JMEExporter;
import com.jme.util.export.JMEImporter;
import com.jme.util.export.OutputCapsule;
import com.jme.util.export.Savable;
import com.jmex.jbullet.nodes.PhysicsNode;

/**
 * <p>PhysicsJoint - Basic jbullet-jme Phyiscs Joint</p>
 * @author normenhansen
 */
public abstract class PhysicsJoint implements Savable {
    protected TypedConstraint constraint;
    protected PhysicsNode nodeA;
    protected PhysicsNode nodeB;
    protected Vector3f pivotA;
    protected Vector3f pivotB;
    protected boolean collisionBetweenLinkedBodys=true;

    public PhysicsJoint(){
    	
    }
    
    /**
     * @param pivotA local translation of the joint connection point in node A
     * @param pivotB local translation of the joint connection point in node A
     */
    public PhysicsJoint(PhysicsNode nodeA, PhysicsNode nodeB, Vector3f pivotA, Vector3f pivotB) {
        this.nodeA = nodeA;
        this.nodeB = nodeB;
        this.pivotA = pivotA;
        this.pivotB = pivotB;
    }

    /**
     * @return the constraint
     */
    public TypedConstraint getConstraint() {
        return constraint;
    }

    /**
     * @return the collisionBetweenLinkedBodys
     */
    public boolean isCollisionBetweenLinkedBodys() {
        return collisionBetweenLinkedBodys;
    }
    
    /**
     * toggles collisions between linked bodys<br>
     * joint has to be removed from and added to PhyiscsSpace to apply this.
     * @param collisionBetweenLinkedBodys set to false to have no collisions between linked bodys
     */
    public void setCollisionBetweenLinkedBodys(boolean collisionBetweenLinkedBodys) {
        this.collisionBetweenLinkedBodys = collisionBetweenLinkedBodys;
    }

    public void destroy(){
    }
    
   

	public PhysicsNode getNodeA() {
		return nodeA;
	}

	public PhysicsNode getNodeB() {
		return nodeB;
	}

	@Override
	public abstract Class getClassTag();

	@Override
	public void read(JMEImporter im) throws IOException {
		InputCapsule capsule = im.getCapsule(this);
		nodeA = (PhysicsNode) capsule.readSavable("nodeA", null);
		nodeB = (PhysicsNode) capsule.readSavable("nodeB", null);
		pivotA = (Vector3f) capsule.readSavable("pivotA", null);
		pivotB = (Vector3f) capsule.readSavable("pivotB", null);
		collisionBetweenLinkedBodys =capsule.readBoolean("collisionBetweenLinkedBodys", true);
	}

	@Override
	public void write(JMEExporter ex) throws IOException {
		OutputCapsule capsule = ex.getCapsule(this);
		capsule.write(nodeA, "nodeA", null);
		capsule.write(nodeB, "nodeB", null);
		capsule.write(pivotA, "pivotA", null);
		capsule.write(pivotB, "pivotB", null);
		capsule.write(collisionBetweenLinkedBodys, "collisionBetweenLinkedBodys", true);
		
	}
}
