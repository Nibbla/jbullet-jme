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

import com.bulletphysics.dynamics.constraintsolver.TypedConstraint;
import com.jme.math.Vector3f;
import com.jmex.jbullet.node.PhysicsNode;

/**
 * <p>PhysicsJoint - Basic jbullet-jme Phyiscs Joint</p>
 * <p>
 * This is the basic way of connecting PhysicsNodes. Although bullet calls these "constraints"
 * in this implementation I call them "joints" because some bullet constraints are
 * used via special PhysicsNodes directly, not via "joints" (e.g. PhysicsVehicleNode).
 * </p>
 * <p>
 * USAGE:<br>
 * TODO: extend javadoc
 * </p>
 * <p>
 * TODO:<br>
 * - a lot
 * </p>
 * @author normenhansen
 */
public class PhysicsJoint {
    protected TypedConstraint constraint;
    protected PhysicsNode nodeA;
    protected PhysicsNode nodeB;
    protected Vector3f pivotA;
    protected Vector3f pivotB;
    private boolean collisionBetweenLinkedBodys=true;
    private boolean update=false;
    private boolean rebuild=false;

    public PhysicsJoint(PhysicsNode nodeA, PhysicsNode nodeB, Vector3f pivotA, Vector3f pivotB) {
        this.nodeA = nodeA;
        this.nodeB = nodeB;
        this.pivotA = pivotA;
        this.pivotB = pivotB;
    }


    public void syncPhysics() {
        
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
     * @param collisionBetweenLinkedBodys the collisionBetweenLinkedBodys to set
     */
    public void setCollisionBetweenLinkedBodys(boolean collisionBetweenLinkedBodys) {
        this.collisionBetweenLinkedBodys = collisionBetweenLinkedBodys;
        update=true;
    }
}
