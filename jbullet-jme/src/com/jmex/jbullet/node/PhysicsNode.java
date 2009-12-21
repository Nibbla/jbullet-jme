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
package com.jmex.jbullet.node;

import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.BvhTriangleMeshShape;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.RigidBodyConstructionInfo;
import com.bulletphysics.linearmath.DefaultMotionState;
import com.bulletphysics.linearmath.MotionState;
import com.bulletphysics.linearmath.Transform;
import com.jme.bounding.BoundingBox;
import com.jme.bounding.BoundingSphere;
import com.jme.bounding.BoundingVolume;
import com.jme.math.Matrix3f;
import com.jme.math.Quaternion;
import com.jme.math.Vector3f;
import com.jme.scene.Node;
import com.jme.scene.Spatial;
import com.jme.scene.TriMesh;
import com.jme.util.export.JMEExporter;
import com.jme.util.export.JMEImporter;
import com.jmex.jbullet.PhysicsSpace;
import com.jmex.jbullet.util.Converter;
import java.io.IOException;

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
 * <p><code>new PhysicsNode(physicsSpace,spatial,shapeType);</code></p>
 * and add it to your jme2 scenegraph.<br>
 * The given spatial will be the added as child node or geometry of the PhysicsNode and
 * will thus move with the PhysicsNode.<br>
 * The shapeType is one of <code>PhyiscsNode.Shapes.XXX</code>
 * and defines what kind of shape the PhysicsNode has in the physics space,
 * the size of the shape is dependent on the BoundingVolume of this PhysicsNode.
 * </p>
 * <p>
 * ISSUES:<br>
 * - getLocalTranslation().set() does not set the physics object location,
 *   use setLocalTranslation(), same applies for getLocalRotation()<br>
 * </p>
 * <p>
 * TODO:<br>
 * - init translation?<br>
 * - separate addToPhyiscsSpace() method?<br>
 * - inertia/mass update strangeness..(probably have to reconstruct RigidBody)<br>
 * - separate update/rebuild<br>
 * - better CollisionShape support (via ModelBound or own Object?)<br>
 * - reuse collision shapes<br>
 * - BoundingVolume update<br>
 * - implement all shapes<br>
 * - update system via MotionStates<br>
 * - phyiscs object update via queue, not syncPhysics() each tick<br>
 * - jme write/read<br>
 * </p>
 * @see com.jmex.jbullet.PhysicsSpace
 * @author normenhansen
 */
public class PhysicsNode extends Node{
    protected PhysicsSpace pSpace;
    protected RigidBody rBody;
    private RigidBodyConstructionInfo constructionInfo;
    private MotionState motionState=new DefaultMotionState();
    private CollisionShape cShape;
    private int shapeType;
    private float mass=1f;
    private float friction=.4f;
    private float angularDamping=.1f;
    private float linearDamping=.1f;
    private float restitution=0f;

    private boolean rebuild=true;

    private boolean physicsEnabled=true;

    //TEMP VARIABLES
    private final Quaternion tmp_inverseWorldRotation = new Quaternion();
    private Transform tempTrans=new Transform();
    private javax.vecmath.Vector3f tempLoc=new javax.vecmath.Vector3f();
    private javax.vecmath.Vector3f tempVel=new javax.vecmath.Vector3f();
    private javax.vecmath.Quat4f tempRot=new javax.vecmath.Quat4f();
    protected com.jme.math.Vector3f tempLocation=new com.jme.math.Vector3f();
    protected com.jme.math.Quaternion tempRotation=new com.jme.math.Quaternion();
    protected com.jme.math.Quaternion tempRotation2=new com.jme.math.Quaternion();
    protected com.jme.math.Matrix3f tempMatrix=new com.jme.math.Matrix3f();

    //TODO: needed here?
    private javax.vecmath.Vector3f localInertia=new javax.vecmath.Vector3f();

    private Vector3f angularVelocity=new Vector3f();

    private Vector3f continuousForce=new Vector3f();

    private Vector3f impulseVector=new Vector3f();
    private float impulseForce=0.0f;

    private boolean applyTranslation=true;
    private boolean applyRotation=true;
    private boolean applyVelocity=true;
    private boolean applyContinuous=false;
    private boolean applyImpulse=false;

    /**
     * Creates a new PhysicsNode with the supplied child node or geometry and
     * creates the standard sphere collision shape for that PhysicsNode<br>
     * @param pSpace
     * @param child
     */
    public PhysicsNode(PhysicsSpace pSpace, Spatial child){
        this(pSpace,child,Shapes.SPHERE);
    }

    /**
     * Creates a new PhysicsNode with the supplied child node or geometry and
     * also creates a collision shape of the given type for that PhysicsNode
     * @param pSpace
     * @param child
     * @param collisionShapeType
     */
    public PhysicsNode(PhysicsSpace pSpace, Spatial child, int collisionShapeType){
        this(pSpace,child,collisionShapeType,1.0f);
    }

    /**
     * Creates a new PhysicsNode with the supplied child node or geometry and
     * also creates a collision shape of the given type for that PhysicsNode and
     * assigns the given mass.
     * @param pSpace
     * @param child
     * @param collisionShapeType
     * @param mass
     */
    public PhysicsNode(PhysicsSpace pSpace, Spatial child, int collisionShapeType, float mass){
        this.pSpace=pSpace;
        this.attachChild(child);
        this.shapeType=collisionShapeType;
        this.mass=mass;
        _createCollisionShape();
        updateRigidBody();
        pSpace.addNode(this);
    }

    /**
     * builds/rebuilds the phyiscs body when parameters have changed
     * TODO:<br>
     * - rebuild mass<br>
     */
    protected void updateRigidBody(){
        if(cShape==null){
            createCollisionSphere();
        }
//        createCollisionShape(shapeType);
		if (mass!=0f) {
            cShape.calculateLocalInertia(mass, localInertia);
		}

        if(rBody!=null){
            //TODO: check if only have to set in constructionInfo
            constructionInfo.collisionShape=cShape;
            constructionInfo.mass=mass;
            constructionInfo.friction=friction;
            constructionInfo.angularDamping=angularDamping;
            constructionInfo.linearDamping=linearDamping;
            constructionInfo.restitution=restitution;

            rBody.setCollisionShape(cShape);
            rBody.setMassProps(mass, localInertia);
            rBody.setFriction(friction);
            rBody.setDamping(angularDamping, linearDamping);
            rBody.setRestitution(restitution);
        }
        else{
            constructionInfo=new RigidBodyConstructionInfo(mass, motionState, cShape, localInertia);
            constructionInfo.friction=friction;
            constructionInfo.angularDamping=angularDamping;
            constructionInfo.linearDamping=linearDamping;
            constructionInfo.restitution=restitution;

            rBody=new RigidBody(constructionInfo);
        }

        switch(shapeType){
            case Shapes.SPHERE:

            break;
            case Shapes.BOX:

            break;
        }
        rebuild=false;
    }

    /**
     * Note that getLocalTranslation().set() will not update the physics object position.
     * Use setLocalTranslation() instead!
     */
    @Override
    public Vector3f getLocalTranslation() {
        return super.getLocalTranslation();
    }

    @Override
    public void setLocalTranslation(Vector3f arg0) {
        super.setLocalTranslation(arg0);
        applyTranslation=true;
    }

    @Override
    public void setLocalTranslation(float arg0, float arg1, float arg2) {
        super.setLocalTranslation(arg0, arg1, arg2);
        applyTranslation=true;
    }

    /**
     * Note that getLocalRotation().set() will not update the physics object position.
     * Use setLocalRotation() instead!
     */
    @Override
    public Quaternion getLocalRotation() {
        return super.getLocalRotation();
    }

    @Override
    public void setLocalRotation(Matrix3f arg0) {
        super.setLocalRotation(arg0);
        applyRotation=true;
    }

    @Override
    public void setLocalRotation(Quaternion arg0) {
        super.setLocalRotation(arg0);
        applyRotation=true;
    }

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

    public void setMass(float mass){
        this.mass=mass;
        rebuild=true;
    }

    public float getFriction() {
        return friction;
    }

    /**
     * sets the friction of this physics object
     * @param friction the friction of this physics object
     */
    public void setFriction(float friction){
        this.friction=friction;
        rebuild=true;
    }

    public float getAngularDamping() {
        return angularDamping;
    }

    public void setAngularDamping(float angularDamping) {
        this.angularDamping = angularDamping;
        rebuild=true;
    }

    public float getLinearDamping() {
        return linearDamping;
    }

    public void setLinearDamping(float linearDamping) {
        this.linearDamping = linearDamping;
        rebuild=true;
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
        rebuild=true;
    }

    public Vector3f getAngularVelocity(){
        return angularVelocity;
    }

    /**
     * sets the angular velocity of this PhysicsNode
     * @param vec the angular velocity of this PhysicsNode
     */
    public void setAngularVelocity(Vector3f vec){
        angularVelocity.set(vec);
        applyVelocity=true;
    }

    public Vector3f getContinuousForce(Vector3f vec){
        if(applyContinuous)
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
        applyContinuous=apply;
    }

    public void applyImpulse(Vector3f vec, float force){
        //TODO: implement
        throw (new UnsupportedOperationException("Not implemented yet."));
    }

    public Vector3f getContinuousTorque(){
        //TODO: implement
        throw(new UnsupportedOperationException("Not implemented yet."));
    }

    /**
     * Apply a continuous torque to this PhysicsNode. The torqu is updated automatically each
     * tick so you only need to set it once and then set it to false to stop applying
     * the torque.
     * @param apply true if the force should be applied each physics tick
     * @param vec the vector of the force to apply
     */
    public void applyContinuousTorque(boolean apply, Vector3f vec){
        //TODO: implement
        throw (new UnsupportedOperationException("Not implemented yet."));
    }

    public void clearForces(){
        //TODO: implement
        throw (new UnsupportedOperationException("Not implemented yet."));
    }

    @Override
    public void updateGeometricState(float arg0, boolean arg1) {
        super.updateGeometricState(arg0, arg1);
    }

    @Override
    public void setModelBound(BoundingVolume modelBound) {
        //TODO: update collision shape from model bound
        super.setModelBound(modelBound);
    }

    public void createCollisionShape(){
        _createCollisionShape();
    }

    public void createCollisionShape(int type){
        shapeType=type;
        _createCollisionShape();
    }

    private void _createCollisionShape(){
        switch(shapeType){
            case Shapes.SPHERE:
                createCollisionSphere();
            break;
            case Shapes.BOX:
                createCollisionBox();
            break;
            case Shapes.CAPSULE:
                createCollisionCapsule();
            break;
            case Shapes.CYLINDER:
                createCollisionCylinder();
            break;
            case Shapes.MESH:
                createCollisionMesh();
            break;
        }
    }

    /**
     * Creates a box in the physics space that represents this Node and all
     * children. The extents are computed from the world bound of this Node.
     */
    private void createCollisionBox() {
        if(children.size()==0){
            throw (new UnsupportedOperationException("PhysicsNode has no children, cannot compute collision box"));
        }
        this.setModelBound(new BoundingBox());
        this.updateModelBound();
        this.updateWorldBound();
        BoundingBox volume=(BoundingBox)this.getWorldBound();
        javax.vecmath.Vector3f halfExtents=new javax.vecmath.Vector3f(volume.xExtent,volume.yExtent,volume.zExtent);
        BoxShape sphere=new BoxShape(halfExtents);
        cShape=sphere;
		if (mass!=0f) {
			cShape.calculateLocalInertia(mass, localInertia);
		}
        rebuild=true;
    }

    /**
     * Creates a sphere in the physics space that represents this Node and all
     * children. The radius is computed from the world bound of this Node.
     */
    private void createCollisionSphere() {
        if(children.size()==0){
            throw (new UnsupportedOperationException("PhysicsNode has no children, cannot compute collision sphere"));
        }
        this.setModelBound(new BoundingSphere());
        this.updateModelBound();
        this.updateWorldBound();
        BoundingSphere volume=(BoundingSphere)this.getWorldBound();
        SphereShape sphere=new SphereShape(volume.getRadius());
        cShape=sphere;
		if (mass!=0f) {
			cShape.calculateLocalInertia(mass, localInertia);
		}
        rebuild=true;
    }

    private void createCollisionCapsule() {
        //TODO: implement
        throw(new UnsupportedOperationException("Not implemented yet."));
    }

    private void createCollisionCylinder(){
        //TODO: implement
        throw(new UnsupportedOperationException("Not implemented yet."));
    }

    /**
     * Creates a mesh that represents this node in the physics space. Can only be
     * used if this Node has one (and only one) TriMesh as a child.<br>
     * TODO: mechanism to reuse collision meshes
     */
    private void createCollisionMesh(){
        if(children.size()==0){
            throw (new UnsupportedOperationException("PhysicsNode has no children, cannot compute collision sphere"));
        }
        else if(children.size()>1){
            throw (new UnsupportedOperationException("Can only create mesh from one single trimesh as leaf in this node."));
        }
        if(getChild(0) instanceof TriMesh){
            TriMesh child=(TriMesh)getChild(0);
            cShape=new BvhTriangleMeshShape(Converter.convert(child),true);
            rebuild=true;
        }
        else{
            throw (new UnsupportedOperationException("No usable trimesh attached to this node!"));
        }
		if (mass!=0f) {
			cShape.calculateLocalInertia(mass, localInertia);
		}
    }
  
    /**
     * called from the main loop to sync jme object with jbullet object
     * TODO: optimize /w list
     */
    public void syncPhysics(){
        if(rebuild) updateRigidBody();
        if(rBody==null) return;

        rBody.getWorldTransform(tempTrans);

        if(applyTranslation){
            tempLocation.set(getWorldTranslation());
            Converter.convert(tempLocation,tempTrans.origin);
            rBody.setWorldTransform(tempTrans);
            motionState.setWorldTransform(tempTrans);
            applyTranslation=false;
        }
        else{
            Converter.convert(tempTrans.origin,tempLocation);
            setWorldTranslation(tempLocation);
        }
        if(applyRotation){
            tempRotation=getWorldRotation();
            Converter.convert(tempRotation, tempRot);
            tempTrans.setRotation(tempRot);
            rBody.setWorldTransform(tempTrans);
            motionState.setWorldTransform(tempTrans);
            applyRotation=false;
        }
        else{
            Converter.convert(tempTrans.basis,tempMatrix);
            tempRotation.fromRotationMatrix(tempMatrix);
            setWorldRotation(tempRotation);
        }
        if(applyVelocity){
            Converter.convert(angularVelocity,tempVel);
            rBody.setAngularVelocity(tempVel);
            applyVelocity=false;
        }
        else{
            Converter.convert(rBody.getAngularVelocity(tempVel),angularVelocity);
        }
        if(applyContinuous){
            //TODO: reuse vector
            rBody.applyCentralForce(Converter.convert(continuousForce));
        }
    }

    /**
     * @return the PhysicsSpace
     */
    public PhysicsSpace getPhysicsSpace() {
        return pSpace;
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
