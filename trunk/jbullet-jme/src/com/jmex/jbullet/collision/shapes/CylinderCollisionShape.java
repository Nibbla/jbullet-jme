/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.jmex.jbullet.collision.shapes;

import com.bulletphysics.collision.shapes.CylinderShape;
import com.bulletphysics.collision.shapes.CylinderShapeZ;
import com.jme.bounding.BoundingBox;
import com.jme.math.Vector3f;
import com.jme.scene.Node;
import com.jme.scene.Spatial;
import com.jmex.jbullet.collision.shapes.CollisionShape.Shapes;
import com.jmex.jbullet.util.Converter;
import java.util.List;

/**
 * Basic cylinder collision shape
 * @author normenhansen
 */
public class CylinderCollisionShape extends CollisionShape{

    /**
     * Creates a collision shape from the bounding volume of the given node
     * @param node the node to get the BoundingVolume from
     */
    public CylinderCollisionShape(Node node) {
        createCollisionCylinder(node);
    }

    /**
     * Creates a collision shape from the given bounding volume
     * @param volume the BoundingVolume to use
     */
    public CylinderCollisionShape(BoundingBox volume) {
        createCollisionCylinder(volume);
    }

    /**
     * Creates a cylinder shape from the given halfextents
     * @param halfExtents
     */
    public CylinderCollisionShape(Vector3f halfExtents) {
        CylinderShape capShape=new CylinderShapeZ(Converter.convert(halfExtents));
        cShape=capShape;
        type=Shapes.CYLINDER;
    }

    private void createCollisionCylinder(Node node){
        List<Spatial> children=node.getChildren();
        if(children.size()==0){
            throw (new UnsupportedOperationException("PhysicsNode has no children, cannot compute collision cylinder"));
        }
        node.setModelBound(new BoundingBox());
        node.updateModelBound();
        node.updateWorldBound();
        BoundingBox volume=(BoundingBox)node.getWorldBound();
        createCollisionCylinder(volume);
    }

    private void createCollisionCylinder(BoundingBox volume){
        javax.vecmath.Vector3f halfExtents=new javax.vecmath.Vector3f(volume.xExtent,volume.yExtent,volume.zExtent);
        CylinderShapeZ capShape=new CylinderShapeZ(halfExtents);
        cShape=capShape;
        type=Shapes.CYLINDER;
    }

}
