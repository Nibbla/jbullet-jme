/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.jmex.jbullet.collision.shapes;

import com.bulletphysics.collision.shapes.BoxShape;
import com.jme.bounding.BoundingBox;
import com.jme.math.Vector3f;
import com.jme.scene.Node;
import com.jme.scene.Spatial;
import com.jmex.jbullet.util.Converter;
import java.util.List;

/**
 * Basic box collision shape
 * @author normenhansen
 */
public class BoxCollisionShape extends CollisionShape{

    /**
     * Creates a collision shape from the bounding volume of the given node
     * @param node the node to get the BoundingVolume from
     */
    public BoxCollisionShape(Node node) {
        createCollisionBox(node);
    }

    /**
     * Creates a collision shape from the given bounding volume
     * @param volume the BoundingVolume to use
     */
    public BoxCollisionShape(BoundingBox volume) {
        createCollisionBox(volume);
    }

    /**
     * Creates a collision box from the given halfExtents
     * @param halfExtents the halfExtents of the CollisionBox
     */
    public BoxCollisionShape(Vector3f halfExtents) {
        BoxShape sphere=new BoxShape(Converter.convert(halfExtents));
        cShape=sphere;
        type=Shapes.BOX;
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
