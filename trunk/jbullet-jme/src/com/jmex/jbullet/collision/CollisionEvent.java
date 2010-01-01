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
package com.jmex.jbullet.collision;

import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.jme.math.Vector3f;
import com.jmex.jbullet.nodes.PhysicsNode;
import com.jmex.jbullet.util.Converter;
import java.util.EventObject;

/**
 * A CollisionEvent stores all information about a collision in the phyiscs world
 * @author normenhansen
 */
public class CollisionEvent extends EventObject{
    public static final int TYPE_ADDED=0;
    public static final int TYPE_PROCESSED=1;
    public static final int TYPE_DESTROYED=1;
    private int type;
    private CollisionObject nodeA;
    private CollisionObject nodeB;

    private ManifoldPoint cp;

//    private float appliedImpulse;
//    private float appliedImpulseLateral1;
//    private float appliedImpulseLateral2;
//    private float combinedFriction;
//    private float combinedRestitution;
//    private float distance1;
//    private int index0;
//    private int index1;
//    private Vector3f lateralFrictionDir1;
//    private Vector3f lateralFrictionDir2;
//    private boolean lateralFrictionInitialized;
//    private int lifeTime;
//    private Vector3f localPointA;
//    private Vector3f localPointB;
//    private Vector3f normalWorldOnB;
//    private int partId0;
//    private int partId1;
//    private Vector3f positionWorldOnA;
//    private Vector3f positionWorldOnB;
//    private Object userPersistentData;
    

//    public CollisionEvent(int type, CollisionObject source, CollisionObject nodeB) {
//        super(source);
//        this.type=type;
//        this.nodeA=source;
//        this.nodeB=nodeB;
//    }

    public CollisionEvent(int type, CollisionObject source, CollisionObject nodeB, ManifoldPoint cp) {
        super(source);
        this.type=type;
        this.nodeA=source;
        this.nodeB=nodeB;
        this.cp=cp;
//        this.appliedImpulse=cp.appliedImpulse;
//        this.appliedImpulseLateral1=cp.appliedImpulseLateral1;
//        this.appliedImpulseLateral2=cp.appliedImpulseLateral2;
//        this.combinedFriction=cp.combinedFriction;
//        this.combinedRestitution=cp.combinedRestitution;
//        this.distance1=cp.distance1;
//        this.index0=cp.index0;
//        this.index1=cp.index1;
//        this.lateralFrictionDir1=Converter.convert(cp.lateralFrictionDir1);
//        this.lateralFrictionDir2=Converter.convert(cp.lateralFrictionDir2);
//        this.lateralFrictionInitialized=cp.lateralFrictionInitialized;
//        this.lifeTime=cp.lifeTime;
//        this.localPointA=Converter.convert(cp.localPointA);
//        this.localPointB=Converter.convert(cp.localPointB);
//        this.normalWorldOnB=Converter.convert(cp.normalWorldOnB);
//        this.partId0=cp.partId0;
//        this.partId1=cp.partId1;
//        this.positionWorldOnA=Converter.convert(cp.positionWorldOnA);
//        this.positionWorldOnB=Converter.convert(cp.positionWorldOnB);
//        this.userPersistentData=cp.userPersistentData;
    }

    public int getType() {
        return type;
    }

    public CollisionObject getNodeA() {
        return nodeA;
    }

    public CollisionObject getNodeB() {
        return nodeB;
    }

    public float getAppliedImpulse() {
        return cp.appliedImpulse;
    }

    public float getAppliedImpulseLateral1() {
        return cp.appliedImpulseLateral1;
    }

    public float getAppliedImpulseLateral2() {
        return cp.appliedImpulseLateral2;
    }

    public float getCombinedFriction() {
        return cp.combinedFriction;
    }

    public float getCombinedRestitution() {
        return cp.combinedRestitution;
    }

    public float getDistance1() {
        return cp.distance1;
    }

    public int getIndex0() {
        return cp.index0;
    }

    public int getIndex1() {
        return cp.index1;
    }

    public Vector3f getLateralFrictionDir1() {
        return Converter.convert(cp.lateralFrictionDir1);
    }

    public Vector3f getLateralFrictionDir2() {
        return Converter.convert(cp.lateralFrictionDir2);
    }

    public boolean isLateralFrictionInitialized() {
        return cp.lateralFrictionInitialized;
    }

    public int getLifeTime() {
        return cp.lifeTime;
    }

    public Vector3f getLocalPointA() {
        return Converter.convert(cp.localPointA);
    }

    public Vector3f getLocalPointB() {
        return Converter.convert(cp.localPointB);
    }

    public Vector3f getNormalWorldOnB() {
        return Converter.convert(cp.normalWorldOnB);
    }

    public int getPartId0() {
        return cp.partId0;
    }

    public int getPartId1() {
        return cp.partId1;
    }

    public Vector3f getPositionWorldOnA() {
        return Converter.convert(cp.positionWorldOnA);
    }

    public Vector3f getPositionWorldOnB() {
        return Converter.convert(cp.positionWorldOnB);
    }

    public Object getUserPersistentData() {
        return cp.userPersistentData;
    }

}