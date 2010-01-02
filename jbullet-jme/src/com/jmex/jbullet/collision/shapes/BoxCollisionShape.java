/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.jmex.jbullet.collision.shapes;

import com.bulletphysics.collision.shapes.BoxShape;
import com.jme.bounding.BoundingBox;
import com.jme.scene.Node;
import com.jme.scene.Spatial;
import java.util.List;

/**
 *
 * @author normenhansen
 */
public class BoxCollisionShape extends CollisionShape{

    public BoxCollisionShape(Node node) {
        createCollisionBox(node);
    }

    public BoxCollisionShape(BoundingBox volume) {
        createCollisionBox(volume);
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
        createCollisionBox(volume);
    }

    private void createCollisionBox(BoundingBox volume) {
        javax.vecmath.Vector3f halfExtents=new javax.vecmath.Vector3f(volume.xExtent,volume.yExtent,volume.zExtent);
        BoxShape sphere=new BoxShape(halfExtents);
        cShape=sphere;
        type=Shapes.BOX;
    }

}
