/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.jmex.jbullet.collision.shapes;

import com.bulletphysics.collision.shapes.SphereShape;
import com.jme.bounding.BoundingSphere;
import com.jme.scene.Node;
import com.jme.scene.Spatial;
import com.jmex.jbullet.collision.shapes.CollisionShape.ShapeTypes;
import java.util.List;

/**
 * Basic sphere collision shape
 * @author normenhansen
 */
public class SphereCollisionShape extends CollisionShape{

    /**
     * Creates a collision shape from the bounding volume of the given node
     * @param node the node to get the BoundingVolume from
     */
    public SphereCollisionShape(Node node) {
        createCollisionSphere(node);
    }

    /**
     * Creates a collision shape from the given bounding volume
     * @param volume the BoundingVolume to use
     */
    public SphereCollisionShape(BoundingSphere volume) {
        createCollisionSphere(volume);
    }

    /**
     * Creates a SphereCollisionShape with the given radius
     * @param radius
     */
    public SphereCollisionShape(float radius) {
        SphereShape sphere=new SphereShape(radius);
        cShape=sphere;
        type=ShapeTypes.SPHERE;
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
        createCollisionSphere(volume);
    }

    private void createCollisionSphere(BoundingSphere volume) {
        SphereShape sphere=new SphereShape(volume.getRadius());
        cShape=sphere;
        type=ShapeTypes.SPHERE;
    }

}
