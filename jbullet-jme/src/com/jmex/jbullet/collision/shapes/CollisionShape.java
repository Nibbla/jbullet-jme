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
package com.jmex.jbullet.collision.shapes;

import javax.vecmath.Vector3f;

/**
 * This Object holds information about a jbullet CollisionShape to be able to reuse
 * CollisionShapes (as suggested in bullet manuals)
 * @author normenhansen
 */
public abstract class CollisionShape {
    protected int type=0;
    protected com.bulletphysics.collision.shapes.CollisionShape cShape;

    public CollisionShape() {
    }

    public int getType() {
        return type;
    }

    /**
     * used internally, not safe
     */
    public void calculateLocalInertia(float mass, Vector3f vector){
        if(cShape==null) return;
        cShape.calculateLocalInertia(mass, vector);
    }

    /**
     * used internally
     */
    public com.bulletphysics.collision.shapes.CollisionShape getCShape() {
        return cShape;
    }

    /**
     * used internally
     */
    public void setCShape(com.bulletphysics.collision.shapes.CollisionShape cShape) {
        this.cShape = cShape;
    }

    /**
     * Interface that contains all jbullet-jme collision shape types.
     */
    public interface Shapes{
        public static final int SPHERE=0;
        public static final int BOX=1;
        public static final int CAPSULE=2;
        public static final int CYLINDER=3;
        public static final int MESH=4;
    }

}
