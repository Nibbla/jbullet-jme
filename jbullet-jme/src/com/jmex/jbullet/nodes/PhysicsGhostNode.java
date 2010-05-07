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

import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.collision.dispatch.GhostObject;
import com.bulletphysics.collision.dispatch.PairCachingGhostObject;
import com.bulletphysics.linearmath.Transform;
import com.jme.math.Matrix3f;
import com.jme.math.Quaternion;
import com.jme.math.Vector3f;
import com.jme.scene.Spatial;
import com.jmex.jbullet.collision.CollisionObject;
import com.jmex.jbullet.collision.shapes.BoxCollisionShape;
import com.jmex.jbullet.collision.shapes.CapsuleCollisionShape;
import com.jmex.jbullet.collision.shapes.CollisionShape;
import com.jmex.jbullet.collision.shapes.CylinderCollisionShape;
import com.jmex.jbullet.collision.shapes.GImpactCollisionShape;
import com.jmex.jbullet.collision.shapes.MeshCollisionShape;
import com.jmex.jbullet.collision.shapes.SphereCollisionShape;
import com.jmex.jbullet.util.Converter;

/**
 * <i>From Bullet manual:</i><br>
 * GhostObject can keep track of all objects that are overlapping.
 * By default, this overlap is based on the AABB.
 * This is useful for creating a character controller,
 * collision sensors/triggers, explosions etc.<br>
 * @author normenhansen
 */
public class PhysicsGhostNode extends CollisionObject {

    protected PairCachingGhostObject gObject;
    protected CollisionShape cShape;
    private boolean physicsEnabled = true;
    //TEMP VARIABLES
    private final Quaternion tmp_inverseWorldRotation = new Quaternion();
    private Transform tempTrans = new Transform();
    private javax.vecmath.Quat4f tempRot = new javax.vecmath.Quat4f();
    protected com.jme.math.Vector3f tempLocation = new com.jme.math.Vector3f();
    protected com.jme.math.Quaternion tempRotation = new com.jme.math.Quaternion();
    protected com.jme.math.Quaternion tempRotation2 = new com.jme.math.Quaternion();
    protected com.jme.math.Matrix3f tempMatrix = new com.jme.math.Matrix3f();

    public PhysicsGhostNode() {
        cShape = new SphereCollisionShape(0.5f);
        buildObject();
    }

    @Deprecated
    public PhysicsGhostNode(Spatial child, int shapeType) {
        this.attachChild(child);
        buildCollisionShape(shapeType);
        buildObject();
    }

    public PhysicsGhostNode(Spatial child, CollisionShape shape) {
        this.attachChild(child);
        cShape = shape;
        buildObject();
    }

    protected void buildObject() {
        if (gObject == null) {
            gObject = new PairCachingGhostObject();
            gObject.setCollisionFlags(gObject.getCollisionFlags() | CollisionFlags.NO_CONTACT_RESPONSE);
        }
        gObject.setCollisionShape(cShape.getCShape());
    }

    /**
     * creates a collisionShape from the BoundingVolume of this node.
     * If no BoundingVolume of the give type exists yet, it will be created.
     * Otherwise a new BoundingVolume will be created.
     * @param type
     */
    private void buildCollisionShape(int type) {
        switch (type) {
            case CollisionShape.ShapeTypes.BOX:
                cShape = new BoxCollisionShape(this);
                break;
            case CollisionShape.ShapeTypes.SPHERE:
                cShape = new SphereCollisionShape(this);
                break;
            case CollisionShape.ShapeTypes.CAPSULE:
                cShape = new CapsuleCollisionShape(this);
                break;
            case CollisionShape.ShapeTypes.CYLINDER:
                cShape = new CylinderCollisionShape(this);
                break;
            case CollisionShape.ShapeTypes.MESH:
                cShape = new MeshCollisionShape(this);
                break;
            case CollisionShape.ShapeTypes.GIMPACT:
                cShape = new GImpactCollisionShape(this);
                break;
        }
    }

    /**
     * note that getLocalTranslation().set() will not update the physics object position.
     * Use setLocalTranslation() instead!
     */
    @Override
    public Vector3f getLocalTranslation() {
        return super.getLocalTranslation();
    }

    /**
     * sets the local translation of this node. The physics object will be updated accordingly
     * in the next global physics update tick.
     * @param arg0
     */
    @Override
    public void setLocalTranslation(Vector3f arg0) {
        super.setLocalTranslation(arg0);
        applyTranslation();
    }

    /**
     * sets the local translation of this node. The physics object will be updated accordingly
     * in the next global physics update tick.
     */
    @Override
    public void setLocalTranslation(float x, float y, float z) {
        super.setLocalTranslation(x, y, z);
        applyTranslation();
    }

    private void applyTranslation() {
        super.updateGeometricState(0, true);
        tempLocation.set(getWorldTranslation());
        gObject.getWorldTransform(tempTrans);
        Converter.convert(tempLocation, tempTrans.origin);
        gObject.setWorldTransform(tempTrans);
    }

    /**
     * note that getLocalRotation().set() will not update the physics object position.
     * Use setLocalRotation() instead!
     */
    @Override
    public Quaternion getLocalRotation() {
        return super.getLocalRotation();
    }

    /**
     * sets the local rotation of this node. The physics object will be updated accordingly
     * in the next global physics update tick.
     * @param arg0
     */
    @Override
    public void setLocalRotation(Matrix3f arg0) {
        super.setLocalRotation(arg0);
        applyRotation();
    }

    /**
     * sets the local rotation of this node. The physics object will be updated accordingly
     * in the next global physics update tick.
     * @param arg0
     */
    @Override
    public void setLocalRotation(Quaternion arg0) {
        super.setLocalRotation(arg0);
        applyRotation();
    }

    @Override
    public void lookAt(Vector3f position, Vector3f upVector) {
        super.lookAt(position, upVector);
        applyRotation();
    }

    @Override
    public void lookAt(Vector3f position, Vector3f upVector, boolean takeParentInAccount) {
        super.lookAt(position, upVector, takeParentInAccount);
        applyRotation();
    }

    @Override
    public void rotateUpTo(Vector3f newUp) {
        super.rotateUpTo(newUp);
        applyRotation();
    }

    private void applyRotation() {
        super.updateGeometricState(0, true);
        tempRotation = getWorldRotation();
        Converter.convert(tempRotation, tempRot);
        gObject.getWorldTransform(tempTrans);
        tempTrans.setRotation(tempRot);
        gObject.setWorldTransform(tempTrans);
    }

    /**
     * computes the local translation from the parameter translation and sets it as new
     * local translation<br>
     * This should only be called from the physics thread to update the jme spatial
     * @param translation new world translation of this spatial.
     * @return the computed local translation
     */
    protected Vector3f setWorldTranslation(Vector3f translation) {
        Vector3f localTranslation = this.getLocalTranslation();
        if (parent != null) {
            localTranslation.set(translation).subtractLocal(parent.getWorldTranslation());
            localTranslation.divideLocal(parent.getWorldScale());
            tmp_inverseWorldRotation.set(parent.getWorldRotation()).inverseLocal().multLocal(localTranslation);
        } else {
            localTranslation.set(translation);
        }
        return localTranslation;
    }

    /**
     * computes the local rotation from the parameter rot and sets it as new
     * local rotation<br>
     * This should only be called from the physics thread to update the jme spatial
     * @param rot new world rotation of this spatial.
     * @return the computed local rotation
     */
    protected Quaternion setWorldRotation(Quaternion rot) {
        Quaternion localRotation = getLocalRotation();
        if (parent != null) {
            tmp_inverseWorldRotation.set(parent.getWorldRotation()).inverseLocal().mult(rot, localRotation);
        } else {
            localRotation.set(rot);
        }
        return localRotation;
    }

    /**
     * note that the physics body and collision shape get
     * rebuilt when scaling this PhysicsNode
     */
    @Override
    public void setLocalScale(float localScale) {
        super.setLocalScale(localScale);
        updateGeometricState(0, true);
        updateWorldBound();
        cShape.setScale(getWorldScale());
    }

    /**
     * note that the physics body and collision shape get
     * rebuilt when scaling this PhysicsNode
     */
    @Override
    public void setLocalScale(Vector3f localScale) {
        super.setLocalScale(localScale);
        updateGeometricState(0, true);
        updateWorldBound();
        cShape.setScale(getWorldScale());
    }

    /**
     * used internally
     */
    public GhostObject getGhostObject() {
        return gObject;
    }

    /**
     * destroys this PhysicsGhostNode and removes it from memory
     */
    public void destroy() {
    }

    public void syncPhysics() {
        if (gObject == null) {
            return;
        }

        gObject.getWorldTransform(tempTrans);

        Converter.convert(tempTrans.origin, tempLocation);
        setWorldTranslation(tempLocation);

        Converter.convert(tempTrans.basis, tempMatrix);
        tempRotation.fromRotationMatrix(tempMatrix);
        setWorldRotation(tempRotation);
    }
}
