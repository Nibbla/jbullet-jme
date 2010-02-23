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
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.linearmath.Transform;
import com.g3d.math.Matrix3f;
import com.g3d.math.Quaternion;
import com.g3d.math.Vector3f;
import com.g3d.scene.Spatial;
import com.jmex.jbullet.PhysicsSpace;
import com.jmex.jbullet.collision.CollisionObject;
import com.jmex.jbullet.collision.shapes.BoxCollisionShape;
import com.jmex.jbullet.collision.shapes.CollisionShape;
import com.jmex.jbullet.nodes.infos.PhysicsNodeState;
import com.jmex.jbullet.util.Converter;
import java.util.concurrent.Callable;

/**
 * <p>PhysicsNode - Basic jbullet-jme physics object</p>
 * @see com.jmex.jbullet.PhysicsSpace
 * @author normenhansen
 */
public class PhysicsNode extends CollisionObject{
    private RigidBodyConstructionInfo constructionInfo;
    protected RigidBody rBody;
    private CollisionShape collisionShape;
    private PhysicsNodeState motionState=new PhysicsNodeState(this);

    private boolean rebuildBody=true;
    private float mass=1.0f;

    //bullet rigidbody properties
    private boolean applyProperties=false;
    private Vector3f gravity=new Vector3f();
    private Vector3f localScale=new Vector3f(1,1,1);
    private float friction=1;
    private float linearDamping=0;
    private float angularDamping=0;
    private float restitution=0;
    private float linearSleepingThreshold=0.8f;
    private float angularSleepingThreshold=1.0f;

    //jme-specific
    private Vector3f continuousForce=new Vector3f();
    private Vector3f continuousForceLocation=new Vector3f();
    private Vector3f continuousTorque=new Vector3f();

    //lock for physics values setting/applying
    private Object physicsLock=new Object();

//    private boolean physicsEnabled=true;

    //TEMP VARIABLES
    private javax.vecmath.Vector3f localInertia=new javax.vecmath.Vector3f();

    private boolean applyForce=false;
    private boolean applyTorque=false;

    private boolean dirty=true;

    public PhysicsNode(){
        collisionShape=new BoxCollisionShape(new Vector3f(0.5f,0.5f,0.5f));
        rebuildBody=true;
    }

    /**
     * creates a new PhysicsNode with the supplied child node or geometry and
     * uses the supplied collision shape for that PhysicsNode<br>
     * @param child
     * @param shape
     */
    public PhysicsNode(Spatial child, CollisionShape shape){
        this(child,shape,1.0f);
    }

    /**
     * creates a new PhysicsNode with the supplied child node or geometry and
     * uses the supplied collision shape for that PhysicsNode<br>
     * @param child
     * @param shape
     */
    public PhysicsNode(Spatial child, CollisionShape shape, float mass){
        this.attachChild(child);
        this.mass=mass;
        this.collisionShape=shape;
        rebuildBody=true;
    }

    /**
     * builds/rebuilds the phyiscs body when parameters have changed
     */
    protected void rebuildRigidBody(){
        boolean removed=false;

//        Transform trans=new Transform();
//        javax.vecmath.Vector3f vec=new javax.vecmath.Vector3f();

        if(rBody!=null){
            System.out.println("rebuild body");
//            rBody.getWorldTransform(trans);
//            rBody.getAngularVelocity(vec);
            if(rBody.isInWorld()){
                PhysicsSpace.getPhysicsSpace().removeDirect(this);
                removed=true;
            }
            rBody.destroy();
        }
        else{
            System.out.println("build body");
        }
        preRebuild();
        rBody=new RigidBody(constructionInfo);
        postRebuild();

        if(removed){
//            rBody.setWorldTransform(trans);
//            rBody.setAngularVelocity(vec);
            PhysicsSpace.getPhysicsSpace().addDirect(this);
        }
    }

    protected void preRebuild(){
//        motionState.setWorldTransform(getWorldTranslation(), getWorldRotation());
        collisionShape.calculateLocalInertia(mass, localInertia);
        if(constructionInfo==null)
            constructionInfo=new RigidBodyConstructionInfo(mass, motionState, collisionShape.getCShape(), localInertia);
        else{
            constructionInfo.mass=mass;
            constructionInfo.collisionShape=collisionShape.getCShape();
            constructionInfo.motionState=motionState;
        }
        constructionInfo.friction=friction;
        constructionInfo.linearDamping=linearDamping;
        constructionInfo.angularDamping=angularDamping;
        constructionInfo.restitution=restitution;
        constructionInfo.linearSleepingThreshold=linearSleepingThreshold;
        constructionInfo.angularSleepingThreshold=angularSleepingThreshold;
    }

    protected void postRebuild(){
        rBody.setUserPointer(this);
        if(mass==0.0f){
            rBody.setCollisionFlags( rBody.getCollisionFlags() | CollisionFlags.STATIC_OBJECT );
        }
        else{
            rBody.setCollisionFlags( rBody.getCollisionFlags() & ~CollisionFlags.STATIC_OBJECT );
        }
    }

//    /**
//     * sets the local translation of this node
//     * @param arg0
//     */
//    @Override
//    public void setLocalTranslation(Vector3f arg0) {
//        super.setLocalTranslation(arg0);
//        applyTranslation();
//    }
//
//    /**
//     * sets the local translation of this node
//     */
//    @Override
//    public void setLocalTranslation(float x, float y, float z) {
//        super.setLocalTranslation(x, y, z);
//        applyTranslation();
//    }
//
//    private void applyTranslation() {
//        setDirty(true);
//    }
//
//    /**
//     * sets the local rotation of this node, the physics object will be updated accordingly
//     * in the next global physics update tick
//     * @param arg0
//     */
//    @Override
//    public void setLocalRotation(Matrix3f arg0) {
//        super.setLocalRotation(arg0);
//        applyRotation();
//    }
//
//    /**
//     * sets the local rotation of this node, the physics object will be updated accordingly
//     * in the next global physics update tick
//     * @param arg0
//     */
//    @Override
//    public void setLocalRotation(Quaternion arg0) {
//        super.setLocalRotation(arg0);
//        applyRotation();
//    }
//
//    @Override
//    public void lookAt(Vector3f position, Vector3f upVector) {
//        super.lookAt(position, upVector);
//        applyRotation();
//    }
//
//    @Override
//    public void rotateUpTo(Vector3f newUp) {
//        super.rotateUpTo(newUp);
//        applyRotation();
//    }
//
//    private void applyRotation() {
//        dirty=true;
//    }

    @Override
    public void setLocalScale(float localScale) {
        super.setLocalScale(localScale);
        this.localScale.set(localScale,localScale,localScale);
        applyProperties=true;
    }

    @Override
    public void setLocalScale(Vector3f localScale) {
        super.setLocalScale(localScale);
        this.localScale.set(localScale);
        applyProperties=true;
    }

    @Override
    protected void setTransformRefresh() {
        super.setTransformRefresh();
        setDirty(true);
    }

    @Override
    public void updateLogicalState(float tpf) {
        super.updateLogicalState(tpf);
    }

    @Override
    public synchronized void updateGeometricState() {
        //else apply user input, dirty flag for physics is set in motionstate
        if(isDirty()){
//            System.out.println("jme transform dirty, apply to motionstate");
            super.updateGeometricState();
            motionState.setWorldTransform(getWorldTranslation(), getWorldRotation());
            setDirty(false);
        }
        //apply physics input, nothing is done if physics did not change
        else {
            if(parent!=null){
                Vector3f localTranslation=getLocalTranslation();
                Quaternion localRotation=getLocalRotation();
                if(motionState.getLocalTransform(getParent(), localTranslation, localRotation)){
//                    System.out.println("apply physics transform "+localTranslation);
                    super.setLocalTranslation(localTranslation);
                    super.setLocalRotation(localRotation);
//                    super.setTransformRefresh();
                    setDirty(false);
                }
            }
            else{
                Vector3f localTranslation=getLocalTranslation();
                Quaternion localRotation=getLocalRotation();
                if(motionState.getWorldTransform(localTranslation, localRotation)){
//                    System.out.println("apply physics transform "+localTranslation);
                    super.setLocalTranslation(localTranslation);
                    super.setLocalRotation(localRotation);
//                    super.setTransformRefresh();
                    setDirty(false);
                }
            }
            super.updateGeometricState();
        }
    }

    //TODO: bwaah.. remove, but its done in such a way that its ridiculously complicated..
    private Transform tempTrans=new Transform();
    /**
     * only to be called from physics thread!!
     */
    public synchronized void updatePhysicsState(){
        if(rebuildBody){
            rebuildRigidBody();
            rebuildBody=false;
        }
        if(motionState.isJmeLocationDirty()){
            rBody.getWorldTransform(tempTrans);
//            System.out.println("apply jme transform "+tempTrans);
            motionState.getWorldTransform(tempTrans);
            rBody.setWorldTransform(tempTrans);
            rBody.activate();
        }
        applyProperties();
    }

    private void applyProperties(){
        if(!applyProperties) return;
//        System.out.println("applying properties");
        collisionShape.getCShape().setLocalScaling(Converter.convert(localScale));
        rBody.setFriction(friction);
        rBody.setDamping(linearDamping, angularDamping);
        rBody.setRestitution(restitution);
        rBody.setSleepingThresholds(linearSleepingThreshold, angularSleepingThreshold);
        rBody.activate();
        applyProperties=false;
    }

    public synchronized float getMass() {
        return mass;
    }

    /**
     * sets the mass of this PhysicsNode, objects with mass=0 are static.
     * @param mass
     */
    public synchronized void setMass(float mass){
        this.mass=mass;
        rebuildBody=true;
    }

    public synchronized void getGravity(Vector3f gravity){
        gravity.set(this.gravity);
        //TODO: gravity
        applyProperties=true;
    }

    /**
     * set the gravity of this PhysicsNode
     * @param gravity the gravity vector to set
     */
    public synchronized void setGravity(Vector3f gravity){
        this.gravity.set(gravity);
        applyProperties=true;
    }

    public synchronized float getFriction() {
        return friction;
    }

    /**
     * sets the friction of this physics object
     * @param friction the friction of this physics object
     */
    public synchronized void setFriction(float friction){
        this.friction=friction;
        applyProperties=true;
    }

    public synchronized void setDamping(float linearDamping,float angularDamping){
        this.linearDamping = linearDamping;
        this.angularDamping = angularDamping;
        applyProperties=true;
    }

    public synchronized float getRestitution() {
        return restitution;
    }

    /**
     * the "bouncyness" of the PhysicsNode
     * best performance if restitution=0
     * @param restitution
     */
    public synchronized void setRestitution(float restitution) {
        this.restitution=restitution;
        applyProperties=true;
    }

    /**
     * get the current angular velocity of this PhysicsNode
     * @return the current linear velocity
     */
    public Vector3f getAngularVelocity(){
        return motionState.getAngularVelocity().clone();
    }

    /**
     * get the current angular velocity of this PhysicsNode
     * @param vec the vector to store the velocity in
     */
    public void getAngularVelocity(Vector3f vec){
        vec.set(motionState.getAngularVelocity());
    }

    /**
     * sets the angular velocity of this PhysicsNode
     * @param vec the angular velocity of this PhysicsNode
     */
    public void setAngularVelocity(Vector3f vec){
        motionState.setAngularVelocity(vec);
    }

    /**
     * get the current linear velocity of this PhysicsNode
     * @return the current linear velocity
     */
    public Vector3f getLinearVelocity(){
        return motionState.getLinearVelocity().clone();
    }

    /**
     * get the current linear velocity of this PhysicsNode
     * @param vec the vector to store the velocity in
     */
    public void getLinearVelocity(Vector3f vec){
        vec.set(motionState.getLinearVelocity());
    }

    /**
     * sets the linear velocity of this PhysicsNode
     * @param vec the linear velocity of this PhysicsNode
     */
    public void setLinearVelocity(Vector3f vec){
        motionState.setLinearVelocity(vec);
    }

    /**
     * get the currently applied continuous force
     * @param vec the vector to store the continuous force in
     * @return null if no force is applied
     */
    public Vector3f getContinuousForce(Vector3f vec){
        if(applyForce)
            return vec.set(continuousForce);
        else
            return null;
    }

    /**
     * get the currently applied continuous force
     * @return null if no force is applied
     */
    public Vector3f getContinuousForce(){
        if(applyForce)
            return continuousForce;
        else
            return null;
    }

    /**
     * get the currently applied continuous force location
     * @return null if no force is applied
     */
    public Vector3f getContinuousForceLocation(){
        if(applyForce)
            return continuousForceLocation;
        else
            return null;
    }

    /**
     * apply a continuous force to this PhysicsNode, the force is updated automatically each
     * tick so you only need to set it once and then set it to false to stop applying
     * the force.
     * @param apply true if the force should be applied each physics tick
     * @param force the vector of the force to apply
     */
    public void applyContinuousForce(boolean apply, Vector3f force){
        if(force!=null) continuousForce.set(force);
        continuousForceLocation.set(0,0,0);
        if(!applyForce&&apply)
            PhysicsSpace.pQueue.add(doApplyContinuousForce);
        applyForce=apply;

    }

    /**
     * apply a continuous force to this PhysicsNode, the force is updated automatically each
     * tick so you only need to set it once and then set it to false to stop applying
     * the force.
     * @param apply true if the force should be applied each physics tick
     * @param force the offset of the force
     */
    public void applyContinuousForce(boolean apply, Vector3f force, Vector3f location){
        if(force!=null) continuousForce.set(force);
        if(location!=null) continuousForceLocation.set(location);
        if(!applyForce&&apply)
            PhysicsSpace.pQueue.add(doApplyContinuousForce);
        applyForce=apply;

    }

    /**
     * use to enable/disable continuous force
     * @param apply set to false to disable
     */
    public void applyContinuousForce(boolean apply){
        if(!applyForce&&apply)
            PhysicsSpace.pQueue.add(doApplyContinuousForce);
        applyForce=apply;
    }

    private Callable doApplyContinuousForce=new Callable(){
        public Object call() throws Exception {
            //TODO: reuse vector
            rBody.applyForce(Converter.convert(continuousForce)
                    ,Converter.convert(continuousForceLocation));
            rBody.activate();
            if(applyForce){
                PhysicsSpace.getPhysicsSpace().reQueue(doApplyContinuousForce);
            }
            return null;
        }

    };

    /**
     * get the currently applied continuous torque
     * @return null if no torque is applied
     */
    public Vector3f getContinuousTorque(){
        if(applyTorque)
            return continuousTorque;
        else
            return null;
    }

    /**
     * get the currently applied continuous torque
     * @param vec the vector to store the continuous torque in
     * @return null if no torque is applied
     */
    public Vector3f getContinuousTorque(Vector3f vec){
        if(applyTorque)
            return vec.set(continuousTorque);
        else
            return null;
    }

    /**
     * apply a continuous torque to this PhysicsNode. The torque is updated automatically each
     * tick so you only need to set it once and then set it to false to stop applying
     * the torque.
     * @param apply true if the force should be applied each physics tick
     * @param vec the vector of the force to apply
     */
    public void applyContinuousTorque(boolean apply, Vector3f vec){
        if(vec!=null) continuousTorque.set(vec);
        if(!applyTorque&&apply){
            PhysicsSpace.pQueue.add(doApplyContinuousTorque);
        }
        applyTorque=apply;
    }

    /**
     * use to enable/disable continuous torque
     * @param apply set to false to disable
     */
    public void applyContinuousTorque(boolean apply){
        if(!applyTorque&&apply){
            PhysicsSpace.pQueue.add(doApplyContinuousTorque);
        }
        applyTorque=apply;
    }

    private Callable doApplyContinuousTorque=new Callable(){
        public Object call() throws Exception {
            //TODO: reuse vector
            rBody.applyTorque(Converter.convert(continuousTorque));
            rBody.activate();
            if(applyTorque){
                PhysicsSpace.getPhysicsSpace().reQueue(doApplyContinuousTorque);
            }
            return null;
        }

    };

    /**
     * apply a force to the PhysicsNode, only applies force in the next physics tick,
     * use applyContinuousForce to apply continuous force
     * @param force the force
     * @param location the location of the force
     */
    public void applyForce(final Vector3f force, final Vector3f location){
        PhysicsSpace.pQueue.add(new Callable(){
            public Object call() throws Exception {
                //TODO: reuse vector!
                rBody.applyForce(Converter.convert(force), Converter.convert(location));
                rBody.activate();
                return null;
            }

        });
    }

    /**
     * apply a force to the PhysicsNode, only applies force in the next physics tick,
     * use applyContinuousForce to apply continuous force
     * @param force the force
     */
    public void applyCentralForce(final Vector3f force){
        PhysicsSpace.pQueue.add(new Callable(){
            public Object call() throws Exception {
                //TODO: reuse vector!
                rBody.applyCentralForce(Converter.convert(force));
                rBody.activate();
                return null;
            }

        });
    }

    /**
     * apply a torque to the PhysicsNode, only applies force in the next physics tick,
     * use applyContinuousTorque to apply continuous torque
     * @param torque the torque
     */
    public void applyTorque(final Vector3f torque){
        PhysicsSpace.pQueue.add(new Callable(){
            public Object call() throws Exception {
                //TODO: reuse vector!
                rBody.applyTorque(Converter.convert(torque));
                rBody.activate();
                return null;
            }

        });
    }

    /**
     * apply an impulse to the PhysicsNode
     * @param vec
     * @param vec2
     */
    public void applyImpulse(final Vector3f vec, final Vector3f vec2){
        PhysicsSpace.pQueue.add(new Callable(){
            public Object call() throws Exception {
                //TODO: reuse vector!
                rBody.applyImpulse(Converter.convert(vec), Converter.convert(vec2));
                rBody.activate();
                return null;
            }

        });
    }

    /**
     * apply a torque impulse to the PhysicsNode
     * @param vec
     */
    public void applyTorqueImpulse(final Vector3f vec){
        PhysicsSpace.pQueue.add(new Callable(){
            public Object call() throws Exception {
                //TODO: reuse vector!
                rBody.applyTorqueImpulse(Converter.convert(vec));
                rBody.activate();
                return null;
            }

        });
    }

    /**
     * clear all forces from the PhysicsNode
     */
    public void clearForces(){
        PhysicsSpace.pQueue.add(new Callable(){
            public Object call() throws Exception {
                rBody.clearForces();
                return null;
            }

        });
    }

    /**
     * @return the CollisionShape of this PhysicsNode, to be able to reuse it with
     * other physics nodes (increases performance)
     */
    public CollisionShape getCollisionShape() {
        return collisionShape;
    }

    /**
     * sets a CollisionShape to be used for this PhysicsNode for reusing CollisionShapes
     * @param collisionShape the CollisionShape to set
     */
    public void setCollisionShape(CollisionShape collisionShape) {
        this.collisionShape = collisionShape;
        rebuildBody=true;
    }

    /**
     * reactivates this PhysicsNode when it has been deactivated because it was not moving
     */
    public void activate(){
        PhysicsSpace.pQueue.add(new Callable(){
            public Object call() throws Exception {
                rBody.activate();
                return null;
            }

        });
    }

    /**
     * sets the sleeping thresholds, these define when the object gets deactivated
     * to save ressources. Low values keep the object active when it barely moves
     * @param linear the linear sleeping threshold
     * @param angular the angular sleeping threshold
     */
    public synchronized void setSleepingThresholds(float linear, float angular){
        this.linearSleepingThreshold=linear;
        this.angularSleepingThreshold=angular;
        applyProperties=true;
//        rBody.setSleepingThresholds(linear, angular);
    }

    /**
     * used internally
     */
    public RigidBody getRigidBody() {
        return rBody;
    }

    /**
     * destroys this PhysicsNode and removes it from memory
     */
    public void destroy(){
        rBody.destroy();
    }

    /**
     * @return the dirty
     */
    public synchronized boolean isDirty() {
        return dirty;
    }

    /**
     * @param dirty the dirty to set
     */
    public synchronized void setDirty(boolean dirty) {
        this.dirty = dirty;
    }

}
