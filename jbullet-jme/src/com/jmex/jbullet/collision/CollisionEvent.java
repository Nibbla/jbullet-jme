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

    private float appliedImpulse;
    private float appliedImpulseLateral1;
    private float appliedImpulseLateral2;
    private float combinedFriction;
    private float combinedRestitution;
    private float distance1;
    private int index0;
    private int index1;
    private Vector3f lateralFrictionDir1;
    private Vector3f lateralFrictionDir2;
    private boolean lateralFrictionInitialized;
    private int lifeTime;
    private Vector3f localPointA;
    private Vector3f localPointB;
    private Vector3f normalWorldOnB;
    private int partId0;
    private int partId1;
    private Vector3f positionWorldOnA;
    private Vector3f positionWorldOnB;
    private Object userPersistentData;
    

    public CollisionEvent(int type, CollisionObject source, CollisionObject nodeB) {
        super(source);
        this.type=type;
        this.nodeA=source;
        this.nodeB=nodeB;
    }

    public CollisionEvent(int type, CollisionObject source, CollisionObject nodeB, ManifoldPoint cp) {
        super(source);
        this.type=type;
        this.nodeA=source;
        this.nodeB=nodeB;
        this.appliedImpulse=cp.appliedImpulse;
        this.appliedImpulseLateral1=cp.appliedImpulseLateral1;
        this.appliedImpulseLateral2=cp.appliedImpulseLateral2;
        this.combinedFriction=cp.combinedFriction;
        this.combinedRestitution=cp.combinedRestitution;
        this.distance1=cp.distance1;
        this.index0=cp.index0;
        this.index1=cp.index1;
        this.lateralFrictionDir1=Converter.convert(cp.lateralFrictionDir1);
        this.lateralFrictionDir2=Converter.convert(cp.lateralFrictionDir2);
        this.lateralFrictionInitialized=cp.lateralFrictionInitialized;
        this.lifeTime=cp.lifeTime;
        this.localPointA=Converter.convert(cp.localPointA);
        this.localPointB=Converter.convert(cp.localPointB);
        this.normalWorldOnB=Converter.convert(cp.normalWorldOnB);
        this.partId0=cp.partId0;
        this.partId1=cp.partId1;
        this.positionWorldOnA=Converter.convert(cp.positionWorldOnA);
        this.positionWorldOnB=Converter.convert(cp.positionWorldOnB);
        this.userPersistentData=cp.userPersistentData;
    }

    public int getType() {
        return type;
    }

    public void setType(int type) {
        this.type = type;
    }

    public CollisionObject getNodeA() {
        return nodeA;
    }

    public void setNodeA(PhysicsNode nodeA) {
        this.nodeA = nodeA;
    }

    public CollisionObject getNodeB() {
        return nodeB;
    }

    public void setNodeB(PhysicsNode nodeB) {
        this.nodeB = nodeB;
    }

    public float getAppliedImpulse() {
        return appliedImpulse;
    }

    public void setAppliedImpulse(float appliedImpulse) {
        this.appliedImpulse = appliedImpulse;
    }

    public float getAppliedImpulseLateral1() {
        return appliedImpulseLateral1;
    }

    public void setAppliedImpulseLateral1(float appliedImpulseLateral1) {
        this.appliedImpulseLateral1 = appliedImpulseLateral1;
    }

    public float getAppliedImpulseLateral2() {
        return appliedImpulseLateral2;
    }

    public void setAppliedImpulseLateral2(float appliedImpulseLateral2) {
        this.appliedImpulseLateral2 = appliedImpulseLateral2;
    }

    public float getCombinedFriction() {
        return combinedFriction;
    }

    public void setCombinedFriction(float combinedFriction) {
        this.combinedFriction = combinedFriction;
    }

    public float getCombinedRestitution() {
        return combinedRestitution;
    }

    public void setCombinedRestitution(float combinedRestitution) {
        this.combinedRestitution = combinedRestitution;
    }

    public float getDistance1() {
        return distance1;
    }

    public void setDistance1(float distance1) {
        this.distance1 = distance1;
    }

    public int getIndex0() {
        return index0;
    }

    public void setIndex0(int index0) {
        this.index0 = index0;
    }

    public int getIndex1() {
        return index1;
    }

    public void setIndex1(int index1) {
        this.index1 = index1;
    }

    public Vector3f getLateralFrictionDir1() {
        return lateralFrictionDir1;
    }

    public void setLateralFrictionDir1(Vector3f lateralFrictionDir1) {
        this.lateralFrictionDir1 = lateralFrictionDir1;
    }

    public Vector3f getLateralFrictionDir2() {
        return lateralFrictionDir2;
    }

    public void setLateralFrictionDir2(Vector3f lateralFrictionDir2) {
        this.lateralFrictionDir2 = lateralFrictionDir2;
    }

    public boolean isLateralFrictionInitialized() {
        return lateralFrictionInitialized;
    }

    public void setLateralFrictionInitialized(boolean lateralFrictionInitialized) {
        this.lateralFrictionInitialized = lateralFrictionInitialized;
    }

    public int getLifeTime() {
        return lifeTime;
    }

    public void setLifeTime(int lifeTime) {
        this.lifeTime = lifeTime;
    }

    public Vector3f getLocalPointA() {
        return localPointA;
    }

    public void setLocalPointA(Vector3f localPointA) {
        this.localPointA = localPointA;
    }

    public Vector3f getLocalPointB() {
        return localPointB;
    }

    public void setLocalPointB(Vector3f localPointB) {
        this.localPointB = localPointB;
    }

    public Vector3f getNormalWorldOnB() {
        return normalWorldOnB;
    }

    public void setNormalWorldOnB(Vector3f normalWorldOnB) {
        this.normalWorldOnB = normalWorldOnB;
    }

    public int getPartId0() {
        return partId0;
    }

    public void setPartId0(int partId0) {
        this.partId0 = partId0;
    }

    public int getPartId1() {
        return partId1;
    }

    public void setPartId1(int partId1) {
        this.partId1 = partId1;
    }

    public Vector3f getPositionWorldOnA() {
        return positionWorldOnA;
    }

    public void setPositionWorldOnA(Vector3f positionWorldOnA) {
        this.positionWorldOnA = positionWorldOnA;
    }

    public Vector3f getPositionWorldOnB() {
        return positionWorldOnB;
    }

    public void setPositionWorldOnB(Vector3f positionWorldOnB) {
        this.positionWorldOnB = positionWorldOnB;
    }

    public Object getUserPersistentData() {
        return userPersistentData;
    }

    public void setUserPersistentData(Object userPersistentData) {
        this.userPersistentData = userPersistentData;
    }

}
