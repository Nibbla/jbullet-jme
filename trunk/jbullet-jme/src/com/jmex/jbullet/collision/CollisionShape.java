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

import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.BvhTriangleMeshShape;
import com.bulletphysics.collision.shapes.CapsuleShape;
import com.bulletphysics.collision.shapes.CylinderShape;
import com.bulletphysics.collision.shapes.SphereShape;
import com.jme.bounding.BoundingBox;
import com.jme.bounding.BoundingCapsule;
import com.jme.bounding.BoundingSphere;
import com.jme.math.FastMath;
import com.jme.scene.Node;
import com.jme.scene.Spatial;
import com.jme.scene.TriMesh;
import com.jmex.jbullet.util.Converter;
import java.util.List;
import javax.vecmath.Vector3f;

/**
 * This Object holds information about a jbullet CollisionShape to be able to reuse
 * CollisionShapes (as suggested in bullet manuals)
 * @author normenhansen
 */
public class CollisionShape {
    private int type=0;
    private com.bulletphysics.collision.shapes.CollisionShape cShape;

    public CollisionShape(int shapeType, Node node) {
        createCollisionShape(shapeType, node);
    }

    public CollisionShape(Node node) {
        createCollisionShape(node);
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

    private void createCollisionShape(Node node){
        if(node.getWorldBound() instanceof BoundingSphere){
            createCollisionSphere(node);
        }
        else if(node.getWorldBound() instanceof BoundingBox){
            createCollisionBox(node);
        }
        else if(node.getWorldBound() instanceof BoundingCapsule){
            createCollisionCapsule(node);
        }
        else{
            createCollisionSphere(node);
        }
    }

    private void createCollisionShape(int shapeType, Node node){
        this.type=shapeType;
        switch(shapeType){
            case Shapes.SPHERE:
                createCollisionSphere(node);
            break;
            case Shapes.BOX:
                createCollisionBox(node);
            break;
            case Shapes.CAPSULE:
                createCollisionCapsule(node);
            break;
            case Shapes.CYLINDER:
                createCollisionCylinder(node);
            break;
            case Shapes.MESH:
                createCollisionMesh(node);
            break;
        }
    }

    /**
     * Creates a box in the physics space that represents this Node and all
     * children. The extents are computed from the world bound of this Node.
     */
    private void createCollisionBox(Node node) {
        List<Spatial> children=node.getChildren();
        if(children.size()==0){
            throw (new UnsupportedOperationException("PhysicsNode has no children, cannot compute collision box"));
        }
        if(!(node.getWorldBound() instanceof BoundingBox)){
            node.setModelBound(new BoundingBox());
            node.updateModelBound();
            node.updateWorldBound();
        }
        BoundingBox volume=(BoundingBox)node.getWorldBound();
        javax.vecmath.Vector3f halfExtents=new javax.vecmath.Vector3f(volume.xExtent,volume.yExtent,volume.zExtent);
        BoxShape sphere=new BoxShape(halfExtents);
        cShape=sphere;
    }

    /**
     * Creates a sphere in the physics space that represents this Node and all
     * children. The radius is computed from the world bound of this Node.
     */
    private void createCollisionSphere(Node node) {
        List<Spatial> children=node.getChildren();
        if(children.size()==0){
            throw (new UnsupportedOperationException("PhysicsNode has no children, cannot compute collision sphere"));
        }
        if(!(node.getWorldBound() instanceof BoundingSphere)){
            node.setModelBound(new BoundingSphere());
            node.updateModelBound();
            node.updateWorldBound();
        }
        BoundingSphere volume=(BoundingSphere)node.getWorldBound();
        SphereShape sphere=new SphereShape(volume.getRadius());
        cShape=sphere;
    }

    private void createCollisionCapsule(Node node) {
        List<Spatial> children=node.getChildren();
        if(children.size()==0){
            throw (new UnsupportedOperationException("PhysicsNode has no children, cannot compute collision capsule"));
        }
        if(!(node.getWorldBound() instanceof BoundingCapsule)){
            node.setModelBound(new BoundingCapsule());
            node.updateModelBound();
            node.updateWorldBound();
        }
        BoundingCapsule capsule=(BoundingCapsule)node.getWorldBound();
        float radius=capsule.getRadius();
        float volume=capsule.getVolume();
        volume-= ( ((4.0f/3.0f) * FastMath.PI ) * FastMath.pow(radius,3) );
        float height=(volume/(FastMath.PI*FastMath.pow(radius,2)));
        height+=(radius*2);
        CapsuleShape capShape=new CapsuleShape(capsule.getRadius(),height);
        cShape=capShape;
    }

    private void createCollisionCylinder(Node node){
        if(5==5)
            throw (new UnsupportedOperationException("Not implemented yet."));
        
        List<Spatial> children=node.getChildren();
        if(children.size()==0){
            throw (new UnsupportedOperationException("PhysicsNode has no children, cannot compute collision cylinder"));
        }
        node.setModelBound(new BoundingBox());
        node.updateModelBound();
        node.updateWorldBound();
        BoundingBox volume=(BoundingBox)node.getWorldBound();
        javax.vecmath.Vector3f halfExtents=new javax.vecmath.Vector3f(volume.xExtent,volume.yExtent,volume.zExtent);
        CylinderShape capShape=new CylinderShape(halfExtents);
        cShape=capShape;
    }

    /**
     * Creates a mesh that represents this node in the physics space. Can only be
     * used if this Node has one (and only one) TriMesh as a child.<br>
     */
    private void createCollisionMesh(Node node){
        List<Spatial> children=node.getChildren();
        if(children.size()==0){
            throw (new UnsupportedOperationException("PhysicsNode has no children, cannot compute collision mesh"));
        }
        else if(children.size()>1){
            throw (new UnsupportedOperationException("Can only create mesh from one single trimesh as leaf in this node."));
        }
        if(node.getChild(0) instanceof TriMesh){
            TriMesh child=(TriMesh)node.getChild(0);
            cShape=new BvhTriangleMeshShape(Converter.convert(child),true);
        }
        else{
            throw (new UnsupportedOperationException("No usable trimesh attached to this node!"));
        }
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
