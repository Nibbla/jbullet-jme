/*
 * Copyright (c) 2009-2010 jMonkeyEngine
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
package com.jmex.jbullet.util;

import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;

import com.jme.bounding.BoundingBox;
import com.jme.math.FastMath;
import com.jme.math.Matrix3f;
import com.jme.math.Quaternion;
import com.jme.math.Vector3f;
import com.jme.scene.Geometry;
import com.jme.scene.Node;
import com.jme.scene.Spatial;
import com.jme.scene.TriMesh;
import com.jme.scene.shape.Arrow;
import com.jme.scene.shape.Box;
import com.jme.scene.shape.Cylinder;
import com.jme.scene.shape.Sphere;
import com.jmex.jbullet.collision.shapes.BoxCollisionShape;
import com.jmex.jbullet.collision.shapes.CapsuleCollisionShape;
import com.jmex.jbullet.collision.shapes.CollisionShape;
import com.jmex.jbullet.collision.shapes.CompoundCollisionShape;
import com.jmex.jbullet.collision.shapes.CylinderCollisionShape;
import com.jmex.jbullet.collision.shapes.GImpactCollisionShape;
import com.jmex.jbullet.collision.shapes.HeightfieldCollisionShape;
import com.jmex.jbullet.collision.shapes.MeshCollisionShape;
import com.jmex.jbullet.collision.shapes.PlaneCollisionShape;
import com.jmex.jbullet.collision.shapes.SphereCollisionShape;
import com.jmex.jbullet.collision.shapes.infos.ChildCollisionShape;

/**
 *
 * @author normenhansen, tim8dev
 */
public class CollisionShapeFactory {

    private static CompoundCollisionShape createCompoundShape(
            Node rootNode, CompoundCollisionShape shape, boolean meshAccurate, boolean dynamic) {
        for (Spatial spatial : rootNode.getChildren()) {
            if (spatial instanceof Node) {
                createCompoundShape((Node) spatial, shape, meshAccurate, dynamic);
            } else if (spatial instanceof TriMesh) {
                if (meshAccurate) {
                    CollisionShape childShape = dynamic
                            ? createSingleDynamicMeshShape((TriMesh) spatial)
                            : createSingleMeshShape((TriMesh) spatial);
                    if (childShape != null) {
                        shape.addChildShape(childShape,
                                spatial.getWorldTranslation(),
                                spatial.getWorldRotation().toRotationMatrix());
                    }
                } else {
                    shape.addChildShape(createSingleBoxShape(spatial),
                            spatial.getWorldTranslation(),
                            spatial.getWorldRotation().toRotationMatrix());
                }
            }
        }
        return shape;
    }

    private static CompoundCollisionShape createCompoundShape(
            Node rootNode, CompoundCollisionShape shape, boolean meshAccurate) {
        if (rootNode.getParent() != null) {
            throw new IllegalStateException("Spatial should not be attached to parent while creating compound collision shape!");
        }
        rootNode.updateGeometricState(0, true);
        return createCompoundShape(rootNode, shape, meshAccurate, false);
    }

    /**
     * This type of collision shape is mesh-accurate and meant for immovable "world objects".
     * Examples include terrain, houses or whole shooter levels.<br>
     * Objects with "mesh" type collision shape will not collide with each other.
     */
    public static CompoundCollisionShape createMeshCompoundShape(Node rootNode) {
        return createCompoundShape(rootNode, new CompoundCollisionShape(), true);
    }

    /**
     * This type of collision shape creates a CompoundShape made out of boxes that
     * are based on the bounds of the Geometries  in the tree.
     * @param rootNode
     * @return
     */
    public static CompoundCollisionShape createBoxCompoundShape(Node rootNode) {
        return createCompoundShape(rootNode, new CompoundCollisionShape(), false);
    }

    /**
     * This type of collision shape is mesh-accurate and meant for immovable "world objects".
     * Examples include terrain, houses or whole shooter levels.<br>
     * Objects with "mesh" type collision shape will not collide with each other.
     * @return A MeshCollisionShape or a CompoundCollisionShape with MeshCollisionShapes as children if the supplied spatial is a Node.
     */
    public static CollisionShape createMeshShape(Spatial spatial) {
        if (spatial instanceof TriMesh) {
            spatial.updateGeometricState(0, true);
            return createSingleMeshShape((TriMesh) spatial);
        } else if (spatial instanceof Node) {
            return createMeshCompoundShape((Node) spatial);
        } else {
            throw new IllegalArgumentException("Supplied spatial must either be Node or Geometry!");
        }
    }

    /**
     * Note that mesh-sccurate dynamic shapes are very expensive and you should consider
     * using a compound shape of standard shapes instead.<br>
     * If you use GImpact shapes, its probably best to do so with a low-poly version of your model.
     * @return A GImpactCollisionShape or a CompoundCollisionShape with GImpactCollisionShapes as children if the supplied spatial is a Node.
     */
    public static CollisionShape createDynamicMeshShape(Spatial spatial) {
        if (spatial instanceof TriMesh) {
            return createSingleDynamicMeshShape((TriMesh) spatial);
        } else if (spatial instanceof Node) {
            if (spatial.getParent() != null) {
                throw new IllegalStateException("Spatial should not be attached to parent while creating compound collision shape!");
            }
            spatial.updateGeometricState(0, true);
            return createCompoundShape((Node) spatial, new CompoundCollisionShape(), true, true);
        } else {
            throw new IllegalArgumentException("Supplied spatial must either be Node or Geometry!");
        }

    }

    public static CollisionShape createBoxShape(Spatial spatial) {
        if (spatial instanceof Geometry) {
            return createSingleBoxShape((Geometry) spatial);
        } else if (spatial instanceof Node) {
            return createBoxCompoundShape((Node) spatial);
        } else {
            throw new IllegalArgumentException("Supplied spatial must either be Node or Geometry!");
        }
    }

    /**
     * This type of collision shape is mesh-accurate and meant for immovable "world objects".
     * Examples include terrain, houses or whole shooter levels.<br>
     * Objects with "mesh" type collision shape will not collide with each other.
     */
    public static MeshCollisionShape createSingleMeshShape(TriMesh mesh) {
        if (mesh != null) {
            MeshCollisionShape mColl = new MeshCollisionShape(mesh);
            mColl.setScale(mesh.getWorldScale());
            return mColl;
        } else {
            return null;
        }
    }

    /**
     * Uses the bounding box of the supplied spatial to create a BoxCollisionShape
     * @param spatial
     * @return BoxCollisionShape with the size of the spatials BoundingBox
     */
    public static BoxCollisionShape createSingleBoxShape(Spatial spatial) {
        spatial.setModelBound(new BoundingBox());
        //TODO: this updateGeometric is not good, it could be called on a spatial
        //      with a parent when compound shapes are created.. why does it crash w/o?
        spatial.updateGeometricState(0, true);
        BoxCollisionShape shape = new BoxCollisionShape(
                ((BoundingBox) spatial.getWorldBound()).getExtent(new Vector3f()));
        return shape;
    }

    /**
     * Note that these mesh-sccurate dynamic shapes (GImpactCollisionShapes) are very expensive and you should consider
     * using a compound shape of standard shapes instead.<br>
     * If you use GImpact shapes, its probably best to do so with a low-poly version of your model.
     */
    public static GImpactCollisionShape createSingleDynamicMeshShape(TriMesh mesh) {
        if (mesh != null) {
            GImpactCollisionShape dynamicShape = new GImpactCollisionShape(mesh);
            dynamicShape.setScale(mesh.getWorldScale());
            return dynamicShape;
        } else {
            return null;
        }
    }

    /**
     * This method moves each child shape of a compound shape by the given vector
     * @param vector
     */
    public static void shiftCompoundShapeContents(CompoundCollisionShape compoundShape, Vector3f vector) {
        for (Iterator<ChildCollisionShape> it = new LinkedList(compoundShape.getChildren()).iterator(); it.hasNext();) {
            ChildCollisionShape childCollisionShape = it.next();
            CollisionShape child = childCollisionShape.shape;
            Vector3f location = childCollisionShape.location;
            Matrix3f rotation = childCollisionShape.rotation;
            compoundShape.removeChildShape(child);
            compoundShape.addChildShape(child, location.add(vector), rotation);
        }
    }

    /**
     * Creates a debug shape from the given collision shape. This is mostly used internally.<br>
     * To attach a debug shape to a physics object, call <code>attachDebugShape(AssetManager manager);</code> on it.
     * @param collisionShape
     * @return
     */
    public static Spatial getDebugShape(CollisionShape collisionShape) {
        if (collisionShape == null) {
            return null;
        }
        Spatial debugShape;
        if (collisionShape instanceof CompoundCollisionShape) {
            CompoundCollisionShape shape = (CompoundCollisionShape) collisionShape;
            List<ChildCollisionShape> children = shape.getChildren();
            Node node = new Node("DebugShapeNode");
            for (Iterator<ChildCollisionShape> it = children.iterator(); it.hasNext();) {
                ChildCollisionShape childCollisionShape = it.next();
                CollisionShape ccollisionShape = childCollisionShape.shape;
                Geometry geometry = createDebugShape(ccollisionShape);

                // apply translation
                geometry.setLocalTranslation(childCollisionShape.location);

                // apply rotation
                TempVars vars = TempVars.get();
                assert vars.lock();
                Matrix3f tempRot = vars.tempMat3;

                tempRot.set(geometry.getLocalRotation());
                childCollisionShape.rotation.mult(tempRot, tempRot);
                geometry.setLocalRotation(tempRot);

                assert vars.unlock();

                node.attachChild(geometry);
            }
            debugShape = node;
        } else {
            debugShape = createDebugShape(collisionShape);
        }
        if (debugShape == null) {
            return null;
        }
        debugShape.updateGeometricState(0, true);
        return debugShape;
    }

    private static Geometry createDebugShape(CollisionShape shape) {
        Geometry geom = null;
        if (shape instanceof BoxCollisionShape) {            
            BoxCollisionShape boxCollisionShape = (BoxCollisionShape) shape;
            final Vector3f halfExtents = boxCollisionShape.getHalfExtents();
            Vector3f scale = boxCollisionShape.getScale();
            geom = new Box("BoxDebugShape", halfExtents.negate(), halfExtents);            
            geom.setLocalScale(scale);
        } else if (shape instanceof SphereCollisionShape) {            
            SphereCollisionShape sphereCollisionShape = (SphereCollisionShape) shape;
            float radius = sphereCollisionShape.getRadius();
            geom = new Sphere("SphereDebugShape", 16, 16, radius);
            Vector3f scale = sphereCollisionShape.getScale();            
            geom.setLocalScale(scale);
        } else if (shape instanceof MeshCollisionShape) {
            MeshCollisionShape meshCollisionShape = (MeshCollisionShape) shape;
            geom = meshCollisionShape.createJmeMesh();
            geom.setName("MeshDebugShape");
            Vector3f scale = meshCollisionShape.getScale();            
            geom.setLocalScale(scale);
        } else if (shape instanceof HeightfieldCollisionShape) {            
            HeightfieldCollisionShape heightfieldCollisionShape = (HeightfieldCollisionShape) shape;
            geom = heightfieldCollisionShape.createJmeMesh();
            geom.setName("HeightfieldDebugShape");
            Vector3f scale = heightfieldCollisionShape.getScale();
            geom.setLocalScale(scale);
        } else if (shape instanceof GImpactCollisionShape) {            
            GImpactCollisionShape meshCollisionShape = (GImpactCollisionShape) shape;
            geom = meshCollisionShape.createJmeMesh();
            geom.setName("GImpactDebugShape");
            Vector3f scale = meshCollisionShape.getScale();            
            geom.setLocalScale(scale);
        } else if (shape instanceof CylinderCollisionShape) {            
            CylinderCollisionShape cylinderCollisionShape = (CylinderCollisionShape) shape;
            Vector3f scale = cylinderCollisionShape.getScale();
            Vector3f halfExtents = cylinderCollisionShape.getHalfExtents();
            int axis = cylinderCollisionShape.getAxis();            
            switch (axis) {
                case 0:
                    geom = new Cylinder("CylinderDebugShape", 16, 16, halfExtents.z, halfExtents.x * 2.0f, true);
                    geom.setLocalRotation(new Quaternion(new float[]{FastMath.HALF_PI, 0, FastMath.HALF_PI}));
                    break;
                case 1:
                    geom = new Cylinder("CylinderDebugShape", 16, 16, halfExtents.x, halfExtents.y * 2.0f, true);
                    geom.setLocalRotation(new Quaternion(new float[]{FastMath.HALF_PI, 0, 0}));
                    break;
                case 2:
                    geom = new Cylinder("CylinderDebugShape", 16, 16, halfExtents.y, halfExtents.z * 2.0f, true);
                    break;
            }            
            geom.setLocalScale(scale);

        } else if (shape instanceof CapsuleCollisionShape) {            
            CapsuleCollisionShape capsuleCollisionShape = (CapsuleCollisionShape) shape;
            Vector3f scale = capsuleCollisionShape.getScale();
            int axis = capsuleCollisionShape.getAxis();
            float height = capsuleCollisionShape.getHeight();
            float radius = capsuleCollisionShape.getRadius();
            //TODO: better debug shape for capsule
            switch (axis) {
                case 0:
                	geom = new Cylinder("CapsuleDebugShape", 16, 16, radius, height + (radius * 2.0f), true);
                    geom.setLocalRotation(new Quaternion(new float[]{FastMath.HALF_PI, 0, FastMath.HALF_PI}));
                    break;
                case 1:
                	geom = new Cylinder("CapsuleDebugShape", 16, 16, radius, height + (radius * 2.0f), true);
                    geom.setLocalRotation(new Quaternion(new float[]{FastMath.HALF_PI, 0, 0}));
                    break;
                case 2:
                	geom = new Cylinder("CapsuleDebugShape", 16, 16, radius, height + (radius * 2.0f), true);
                    break;
            }            
            geom.setLocalScale(scale);
        } 
        return geom;
    }
}