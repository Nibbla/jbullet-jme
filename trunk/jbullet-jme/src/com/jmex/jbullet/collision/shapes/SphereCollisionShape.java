/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.jmex.jbullet.collision.shapes;

import com.bulletphysics.collision.shapes.SphereShape;
import com.jme.bounding.BoundingSphere;
import com.jme.scene.Node;
import com.jme.scene.Spatial;
import com.jmex.jbullet.collision.shapes.CollisionShape.Shapes;
import java.util.List;

/**
 *
 * @author normenhansen
 */
public class SphereCollisionShape extends CollisionShape{

    public SphereCollisionShape(Node node) {
        createCollisionSphere(node);
    }

    public SphereCollisionShape(BoundingSphere volume) {
        createCollisionSphere(volume);
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
        type=Shapes.SPHERE;
    }

}
