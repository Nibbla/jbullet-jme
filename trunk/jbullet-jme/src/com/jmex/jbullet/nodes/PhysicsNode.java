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

import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.linearmath.MotionState;
import com.bulletphysics.linearmath.Transform;
import com.jme.bounding.BoundingVolume;
import com.jme.math.Matrix3f;
import com.jme.math.Quaternion;
import com.jme.math.Vector3f;
import com.jme.scene.Spatial;
import com.jme.util.GameTaskQueue;
import com.jme.util.GameTaskQueueManager;
import com.jme.util.export.JMEExporter;
import com.jme.util.export.JMEImporter;
import com.jmex.jbullet.PhysicsSpace;
import com.jmex.jbullet.collision.CollisionObject;
import com.jmex.jbullet.collision.CollisionShape;
import com.jmex.jbullet.collision.CollisionShape.Shapes;
import com.jmex.jbullet.util.Converter;
import java.io.IOException;
import java.util.concurrent.Callable;

/**
 * <p>PhysicsNode - Basic jbullet-jme physics object</p>
 * <p>
 * A PhysicsNode represents an object in the physics space and in the jme scenegraph.
 * It is derived from <code>com.jme.scene.Node</code> to integrate nicely with the jme2 scenegraph.<br>
 * For bullet, it stores a RigidBody and its CollisionShape including their options (friction, mass etc.)<br>
 * </p>
 * <p>
 * USAGE:<br>
 * To create a PhysicsNode, just create a new PhysicsNode Object like this
 * <p><code>new PhysicsNode(spatial,shapeType);</code></p>
 * and add it to your jme2 scenegraph.<br>
 * Then add the PhyiscsNode to the PhysicsSpace by calling <code>physicsSpace.add(node);</code>
 * The given spatial will be the added as child node or geometry of the PhysicsNode and
 * will thus move with the PhysicsNode.<br>
 * The shapeType is one of <code>CollisionShape.Shapes.XXX</code>
 * and defines what kind of shape the PhysicsNode has in the physics space,
 * the size of the shape is dependent on the BoundingVolume of this PhysicsNode.<br>
 * PhysicsNodes with mass=0 are static and do not move.
 * </p>
 * <p>
 * HINTS:<br>
 * - It is not recommended to have PhysicsNodes as child nodes of other PhysicsNodes<br>
 * - getLocalTranslation().set() does not set the physics object location,
 *   use setLocalTranslation(), same applies for getLocalRotation()<br>
 * </p>
 * @see com.jmex.jbullet.PhysicsSpace
 * @author normenhansen
 */
public class PhysicsNode extends CollisionObject{
    protected RigidBody rBody;
    private RigidBodyConstructionInfo constructionInfo;
    private MotionState motionState;//=new DefaultMotionState();
    private CollisionShape collisionShape;
    private float mass=1f;
    private float friction=1f;//.4f;
    private float angularDamping=.1f;
    private float linearDamping=.1f;
    private float restitution=0f;

    private boolean physicsEnabled=true;

    //TEMP VARIABLES
    private final Quaternion tmp_inverseWorldRotation = new Quaternion();
    private Transform tempTrans=new Transform();
    private javax.vecmath.Vector3f tempVel=new javax.vecmath.Vector3f();
    private javax.vecmath.Quat4f tempRot=new javax.vecmath.Quat4f();
    private com.jme.math.Vector3f tempLocation=new com.jme.math.Vector3f();
    private com.jme.math.Quaternion tempRotation=new com.jme.math.Quaternion();
    private com.jme.math.Matrix3f tempMatrix=new com.jme.math.Matrix3f();

    //TODO: needed here?
    private javax.vecmath.Vector3f localInertia=new javax.vecmath.Vector3f();

    private Vector3f angularVelocity=new Vector3f();

    private Vector3f gravity=new Vector3f();

    private Vector3f continuousForce=new Vector3f();

    private Vector3f continuousTorque=new Vector3f();

    private Vector3f impulseVector=new Vector3f();
    private Vector3f impulseVector2=new Vector3f();

    private Vector3f torqueImpulse=new Vector3f();

    protected GameTaskQueue pQueue=GameTaskQueueManager.getManager().getQueue("jbullet_sync");
    private boolean applyForce=false;
    private boolean applyTorque=false;

    /**
     * Creates a new PhysicsNode with the supplied child node or geometry and
     * creates the standard sphere collision shape for that PhysicsNode<br>
     * @param child
     */
    public PhysicsNode(Spatial child){
        this(child,Shapes.SPHERE);
    }

    /**
     * Creates a new PhysicsNode with the supplied child node or geometry and
     * also creates a collision shape of the given type for that PhysicsNode
     * @param child
     * @param collisionShapeType
     */
    public PhysicsNode(Spatial child, int collisionShapeType){
        this(child,collisionShapeType,1.0f);
    }

    /**
     * Creates a new PhysicsNode with the supplied child node or geometry and
     * also creates a collision shape of the given type for that PhysicsNode and
     * assigns the given mass.
     * @param child
     * @param collisionShapeType
     * @param mass
     */
    public PhysicsNode(Spatial child, int collisionShapeType, float mass){
        this.attachChild(child);
        this.mass=mass;
        motionState=createMotionState();
        createCollisionShape(collisionShapeType);
    }

    /**
     * Creates a new PhysicsNode with the supplied child node or geometry and
     * uses the supplied collision shape for that PhysicsNode<br>
     * @param child
     * @param shape
     */
    public PhysicsNode(Spatial child, CollisionShape shape){
        this(child,shape,1.0f);
    }

    /**
     * Creates a new PhysicsNode with the supplied child node or geometry and
     * uses the supplied collision shape for that PhysicsNode<br>
     * @param child
     * @param shape
     */
    public PhysicsNode(Spatial child, CollisionShape shape, float mass){
        this.attachChild(child);
        this.mass=mass;
        this.collisionShape=shape;
        motionState=createMotionState();
        rebuildRigidBody();
    }

    protected MotionState createMotionState(){
        return new MotionState(){

            public Transform getWorldTransform(Transform out) {
                if(out==null)
                    out=new Transform();

                tempRotation.set(getWorldRotation());
                Converter.convert(tempRotation, tempRot);

                out.basis.set(tempRot);
                out.origin.set(Converter.convert(getWorldTranslation()));
                return out;
            }

            public void setWorldTransform(Transform worldTrans) {
                //TODO: reuse transform
                final Transform trans=new Transform(worldTrans);

                pQueue.enqueue(new Callable(){

                    public Object call() throws Exception {

                        Converter.convert(trans.origin,tempLocation);
                        setWorldTranslation(tempLocation);

                        Converter.convert(trans.basis,tempMatrix);
                        tempRotation.fromRotationMatrix(tempMatrix);
                        setWorldRotation(tempRotation);

                        Converter.convert(rBody.getAngularVelocity(tempVel),angularVelocity);
                        return null;
                    }
                });
            }

        };
    }

    /**
     * updates the phyiscs body when parameters have changed
     */
    protected void updateRigidBody(){
        if(rBody!=null){
            //TODO: check if only have to set in constructionInfo
            updateConstructionInfo();
//            rBody.setCollisionShape(collisionShape.getCShape());
//            rBody.setMassProps(mass, localInertia);
            rBody.setFriction(friction);
            rBody.setDamping(angularDamping, linearDamping);
            rBody.setRestitution(restitution);
//            rBody.setGravity(Converter.convert(gravity));
        }
        else{
            System.out.println("Error - RigidBody is null!");
        }
    }

    private Callable doUpdate=new Callable(){
        public Object call() throws Exception {
            updateRigidBody();
            return null;
        }
    };

    /**
     * builds/rebuilds the phyiscs body when parameters have changed
     */
    protected void rebuildRigidBody(){
        boolean removed=false;

        Transform trans=new Transform();
        javax.vecmath.Vector3f vec=new javax.vecmath.Vector3f();

        if(rBody!=null){
            System.out.println("rebuild");
            rBody.getWorldTransform(trans);
            rBody.getAngularVelocity(vec);
            PhysicsSpace.getPhysicsSpace().remove(this);
            rBody.destroy();
            removed=true;
        }

//        if(collisionShape==null) System.out.println("shitt!!");

        preRebuild();
        rBody=new RigidBody(constructionInfo);
        postRebuild();

        if(removed){
            rBody.setWorldTransform(trans);
            rBody.setAngularVelocity(vec);
            PhysicsSpace.getPhysicsSpace().add(this);
        }
    }

    private Callable doRebuild=new Callable(){
        public Object call() throws Exception {
            rebuildRigidBody();
            return null;
        }
    };

    protected void preRebuild(){
        collisionShape.calculateLocalInertia(mass, localInertia);
        constructionInfo=new RigidBodyConstructionInfo(mass, motionState, collisionShape.getCShape(), localInertia);
        updateConstructionInfo();
    }

    protected void postRebuild(){

    }

    private void updateConstructionInfo(){
        constructionInfo.collisionShape=collisionShape.getCShape();
        constructionInfo.mass=mass;
        constructionInfo.friction=friction;
        constructionInfo.angularDamping=angularDamping;
        constructionInfo.linearDamping=linearDamping;
        constructionInfo.restitution=restitution;
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

    private void applyTranslation() {
        tempLocation.set(getWorldTranslation());
        Converter.convert(tempLocation,tempTrans.origin);
        rBody.setWorldTransform(tempTrans);
    }

    private Callable doApplyTranslation=new Callable(){
        public Object call() throws Exception {
            applyTranslation();
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
        if(rBody!=null&&rBody.isInWorld())
            pQueue.enqueue(doApplyRotation);
        else
            applyRotation();
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

    private void applyRotation() {
        tempRotation.set(getWorldRotation());
        Converter.convert(tempRotation, tempRot);
        tempTrans.setRotation(tempRot);
        rBody.setWorldTransform(tempTrans);
    }

    private Callable doApplyRotation=new Callable(){
        public Object call() throws Exception {
            applyRotation();
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

    public float getMass() {
        return mass;
    }

    /**
     * Sets the mass of this PhysicsNode, objects with mass=0 are static.
     * @param mass
     */
    public void setMass(float mass){
        this.mass=mass;
        if(rBody!=null&&rBody.isInWorld())
            pQueue.enqueue(doRebuild);
        else
            rebuildRigidBody();
    }

    public void getGravity(Vector3f gravity){
        gravity.set(this.gravity);
    }

    public void setGravity(Vector3f gravity){
        this.gravity.set(gravity);
        if(rBody!=null&&rBody.isInWorld())
            pQueue.enqueue(doApplyGravity);
        else
            applyGravity();
    }

    private void applyGravity() {
        rBody.setGravity(Converter.convert(gravity));
    }

    private Callable doApplyGravity=new Callable(){
        public Object call() throws Exception {
            applyGravity();
            return null;
        }
    };

    public float getFriction() {
        return friction;
    }

    /**
     * sets the friction of this physics object
     * @param friction the friction of this physics object
     */
    public void setFriction(float friction){
        this.friction=friction;
        if(rBody!=null&&rBody.isInWorld())
            pQueue.enqueue(doUpdate);
        else
            updateRigidBody();
    }

    public void setDamping(float linearDamping,float angularDamping){
        this.linearDamping = linearDamping;
        this.angularDamping = angularDamping;
        if(rBody!=null&&rBody.isInWorld())
            pQueue.enqueue(doUpdate);
        else
            updateRigidBody();
    }

    public float getAngularDamping() {
        return angularDamping;
    }

    public void setAngularDamping(float angularDamping) {
        this.angularDamping = angularDamping;
        if(rBody!=null&&rBody.isInWorld())
            pQueue.enqueue(doUpdate);
        else
            updateRigidBody();
    }

    public float getLinearDamping() {
        return linearDamping;
    }

    public void setLinearDamping(float linearDamping) {
        this.linearDamping = linearDamping;
        if(rBody!=null&&rBody.isInWorld())
            pQueue.enqueue(doUpdate);
        else
            updateRigidBody();
    }

    public float getRestitution() {
        return restitution;
    }

    /**
     * best performance if restitution=0
     * @param restitution
     */
    public void setRestitution(float restitution) {
        this.restitution = restitution;
        if(rBody!=null&&rBody.isInWorld())
            pQueue.enqueue(doUpdate);
        else
            updateRigidBody();
    }

    public Vector3f getAngularVelocity(){
        return angularVelocity;
    }

    public void getAngularVelocity(Vector3f vec){
        vec.set(angularVelocity);
    }

    /**
     * sets the angular velocity of this PhysicsNode
     * @param vec the angular velocity of this PhysicsNode
     */
    public void setAngularVelocity(Vector3f vec){
        angularVelocity.set(vec);
        if(rBody!=null&&rBody.isInWorld())
            pQueue.enqueue(doApplyVelocity);
        else
            applyVelocity();
    }

    private void applyVelocity() {
        Converter.convert(angularVelocity,tempVel);
        rBody.setAngularVelocity(tempVel);
    }

    private Callable doApplyVelocity=new Callable(){
        public Object call() throws Exception {
            applyVelocity();
            return null;
        }
    };

    public Vector3f getContinuousForce(Vector3f vec){
        if(applyForce)
            return vec.set(continuousForce);
        else
            return null;
    }

    public Vector3f getContinuousForce(){
        if(applyForce)
            return continuousForce;
        else
            return null;
    }

    /**
     * Apply a continuous force to this PhysicsNode. The force is updated automatically each
     * tick so you only need to set it once and then set it to false to stop applying
     * the force.
     * @param apply true if the force should be applied each physics tick
     * @param vec the vector of the force to apply
     */
    public void applyContinuousForce(boolean apply, Vector3f vec){
        if(vec!=null) continuousForce.set(vec);
        applyForce=apply;
    }

    public void applyImpulse(Vector3f vec, Vector3f vec2){
        if(vec!=null) impulseVector.set(vec);
        if(vec2!=null) impulseVector2.set(vec);
        if(rBody!=null&&rBody.isInWorld())
            pQueue.enqueue(doApplyImpulse);
        else
            applyImpulse();
    }

    private void applyImpulse() {
        //TODO: reuse vector
        rBody.applyImpulse(Converter.convert(impulseVector), Converter.convert(impulseVector2));
    }

    private Callable doApplyImpulse=new Callable(){
        public Object call() throws Exception {
            applyImpulse();
            return null;
        }
    };

    public void applyTorqueImpulse(Vector3f vec){
        if(vec!=null) torqueImpulse.set(vec);
        pQueue.enqueue(doApplyTorqueImpulse);
    }

    private void applyTorqueImpulse() {
        //TODO: reuse vector
        rBody.applyTorqueImpulse(Converter.convert(torqueImpulse));
    }

    private Callable doApplyTorqueImpulse=new Callable(){
        public Object call() throws Exception {
            applyTorqueImpulse();
            return null;
        }
    };

    public Vector3f getContinuousTorque(){
        return continuousTorque;
    }

    public Vector3f getContinuousTorque(Vector3f vec){
        return vec.set(continuousTorque);
    }

    /**
     * Apply a continuous torque to this PhysicsNode. The torque is updated automatically each
     * tick so you only need to set it once and then set it to false to stop applying
     * the torque.
     * @param apply true if the force should be applied each physics tick
     * @param vec the vector of the force to apply
     */
    public void applyContinuousTorque(boolean apply, Vector3f vec){
        if(vec!=null) continuousTorque.set(vec);
        applyTorque=apply;
    }

    public void clearForces(){
        if(rBody!=null&&rBody.isInWorld())
            pQueue.enqueue(doClearForces);
        else
            rBody.clearForces();
    }

    private Callable doClearForces=new Callable(){
        public Object call() throws Exception {
            //TODO: reuse vector
            rBody.clearForces();
            return null;
        }
    };

    @Override
    public void updateGeometricState(float arg0, boolean arg1) {
        super.updateGeometricState(arg0, arg1);
    }

    @Override
    public void setModelBound(BoundingVolume modelBound) {
        //TODO: update collision shape from model bound
        super.setModelBound(modelBound);
    }

    /**
     * creates a collisionShape from the BoundingVolume of this node.
     * If no BoundingVolume of the give type exists yet, it will be created.
     * Otherwise a new BoundingVolume will be created.
     * @param type
     */
    public void createCollisionShape(int type){
        collisionShape=new CollisionShape(type, this);
        if(rBody!=null&&rBody.isInWorld())
            pQueue.enqueue(doRebuild);
        else
            rebuildRigidBody();
    }
  
    /**
     * creates a collisionShape from the current BoundingVolume of this node.
     * If no BoundingVolume of a proper type (box, sphere, cyliner, capsule) exists,
     * a sphere will be created.
     */
    public void createCollisionShape(){
        collisionShape=new CollisionShape(this);
        if(rBody!=null&&rBody.isInWorld())
            pQueue.enqueue(doRebuild);
        else
            rebuildRigidBody();
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
        if(rBody!=null&&rBody.isInWorld())
            pQueue.enqueue(doRebuild);
        else
            rebuildRigidBody();
    }

    /**
     * called from the main loop to sync jme object with jbullet object<br>
     * TODO: replace /w queue
     */
    public void syncPhysics(){
        if(rBody==null) return;
        //if body is not in world, apply all jme infos
        if(!rBody.isInWorld()){
            updateRigidBody();
            applyTranslation();
            applyRotation();
        }

        if(applyForce){
            //TODO: reuse vector
            rBody.applyCentralForce(Converter.convert(continuousForce));
        }
        if(applyTorque){
            //TODO: reuse vector
            rBody.applyTorque(Converter.convert(continuousTorque));
        }
    }

    /**
     * @return the JBullet RigidBody
     */
    public RigidBody getRigidBody() {
        if(rBody==null)
            updateRigidBody();
        return rBody;
    }

    @Override
    public void write(JMEExporter e) throws IOException {
        throw (new UnsupportedOperationException("Not implemented yet."));
    }

    @Override
    public void read(JMEImporter e) throws IOException {
        throw (new UnsupportedOperationException("Not implemented yet."));
    }

}
