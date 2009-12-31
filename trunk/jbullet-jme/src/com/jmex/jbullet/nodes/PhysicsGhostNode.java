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
package com.jmex.jbullet.nodes;

import com.bulletphysics.collision.dispatch.GhostObject;
import com.bulletphysics.collision.dispatch.PairCachingGhostObject;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.MotionState;
import com.bulletphysics.linearmath.Transform;
import com.jme.math.Matrix3f;
import com.jme.math.Quaternion;
import com.jme.math.Vector3f;
import com.jme.scene.Spatial;
import com.jme.util.GameTaskQueue;
import com.jme.util.GameTaskQueueManager;
import com.jmex.jbullet.collision.CollisionObject;
import com.jmex.jbullet.collision.CollisionShape;
import com.jmex.jbullet.util.Converter;
import java.util.concurrent.Callable;

/**
 * <i>From Bullet manual:</i><br>
 * GhostObject can keep track of all objects that are overlapping.
 * By default, this overlap is based on the AABB.
 * This is useful for creating a character controller,
 * collision sensors/triggers, explosions etc.<br>
 * @author normenhansen
 */
public class PhysicsGhostNode extends CollisionObject{
	protected PairCachingGhostObject gObject;
    protected CollisionShape cShape;

    private boolean physicsEnabled=true;

    //TEMP VARIABLES
    private final Quaternion tmp_inverseWorldRotation = new Quaternion();
    private Transform tempTrans=new Transform();
    private javax.vecmath.Quat4f tempRot=new javax.vecmath.Quat4f();
    protected com.jme.math.Vector3f tempLocation=new com.jme.math.Vector3f();
    protected com.jme.math.Quaternion tempRotation=new com.jme.math.Quaternion();
    protected com.jme.math.Quaternion tempRotation2=new com.jme.math.Quaternion();
    protected com.jme.math.Matrix3f tempMatrix=new com.jme.math.Matrix3f();

    protected GameTaskQueue pQueue=GameTaskQueueManager.getManager().getQueue("jbullet_sync");

    public PhysicsGhostNode(Spatial child, int shapeType) {
        this.attachChild(child);
        buildCollisionShape(shapeType);
        buildObject();
    }

    public PhysicsGhostNode(Spatial child, CollisionShape shape){
        this.attachChild(child);
        cShape=shape;
        buildObject();
    }

    protected void buildObject() {
        gObject=new PairCachingGhostObject();
        gObject.setCollisionShape(cShape.getCShape());
    }

    private void buildCollisionShape(int type){
        cShape=new CollisionShape(type, this);
    }


    /**
     * Note that getLocalTranslation().set() will not update the physics object position.
     * Use setLocalTranslation() instead!
     */
    @Override
    public Vector3f getLocalTranslation() {
        return super.getLocalTranslation();
    }

    /**
     * Sets the local translation of this node. The physics object will be updated accordingly
     * in the next global physics update tick.
     * @param arg0
     */
    @Override
    public void setLocalTranslation(Vector3f arg0) {
        super.setLocalTranslation(arg0);
        pQueue.enqueue(doApplyTranslation);
    }

    /**
     * Sets the local translation of this node. The physics object will be updated accordingly
     * in the next global physics update tick.
     */
    @Override
    public void setLocalTranslation(float x, float y, float z) {
        super.setLocalTranslation(x, y, z);
        pQueue.enqueue(doApplyTranslation);
    }

    private Callable doApplyTranslation=new Callable(){
        public Object call() throws Exception {
            tempLocation.set(getWorldTranslation());
            Converter.convert(tempLocation,tempTrans.origin);
            gObject.setWorldTransform(tempTrans);
            return null;
        }
    };

    /**
     * Note that getLocalRotation().set() will not update the physics object position.
     * Use setLocalRotation() instead!
     */
    @Override
    public Quaternion getLocalRotation() {
        return super.getLocalRotation();
    }

    /**
     * Sets the local rotation of this node. The physics object will be updated accordingly
     * in the next global physics update tick.
     * @param arg0
     */
    @Override
    public void setLocalRotation(Matrix3f arg0) {
        super.setLocalRotation(arg0);
        pQueue.enqueue(doApplyRotation);
    }

    /**
     * Sets the local rotation of this node. The physics object will be updated accordingly
     * in the next global physics update tick.
     * @param arg0
     */
    @Override
    public void setLocalRotation(Quaternion arg0) {
        super.setLocalRotation(arg0);
        pQueue.enqueue(doApplyRotation);
    }

    private Callable doApplyRotation=new Callable(){
        public Object call() throws Exception {
            tempRotation=getWorldRotation();
            Converter.convert(tempRotation, tempRot);
            tempTrans.setRotation(tempRot);
            gObject.setWorldTransform(tempTrans);
            return null;
        }
    };

    /**
     * Computes the local translation from the parameter translation and sets it as new
     * local translation.<br>
     * This should only be called from the physics thread to update the jme spatial
     * @param translation new world translation of this spatial.
     * @return the computed local translation
     */
    public Vector3f setWorldTranslation( Vector3f translation ) {
        Vector3f localTranslation = this.getLocalTranslation();
        if ( parent != null ) {
            localTranslation.set( translation ).subtractLocal(parent.getWorldTranslation() );
            localTranslation.divideLocal( parent.getWorldScale() );
            tmp_inverseWorldRotation.set( parent.getWorldRotation()).inverseLocal().multLocal( localTranslation );
        }
        else {
            localTranslation.set( translation );
        }
        return localTranslation;
    }

    /**
     * Computes the local rotation from the parameter rot and sets it as new
     * local rotation.<br>
     * This should only be called from the physics thread to update the jme spatial
     * @param rot new world rotation of this spatial.
     * @return the computed local rotation
     */
    public Quaternion setWorldRotation( Quaternion rot ) {
        Quaternion localRotation = getLocalRotation();
        if ( parent != null ) {
            tmp_inverseWorldRotation.set( parent.getWorldRotation()).inverseLocal().mult( rot, localRotation );
        }
        else {
            localRotation.set( rot );
        }
        return localRotation;
    }

    public GhostObject getGhostObject(){
        return gObject;
    }

    public void syncPhysics(){
        if(gObject==null) return;

        gObject.getWorldTransform(tempTrans);

        Converter.convert(tempTrans.origin,tempLocation);
        setWorldTranslation(tempLocation);

        Converter.convert(tempTrans.basis,tempMatrix);
        tempRotation.fromRotationMatrix(tempMatrix);
        setWorldRotation(tempRotation);
    }

}
