/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.jmex.jbullet.collision.shapes;

import com.bulletphysics.collision.shapes.CylinderShape;
import com.jme.bounding.BoundingBox;
import com.jme.scene.Node;
import com.jme.scene.Spatial;
import com.jmex.jbullet.collision.shapes.CollisionShape.Shapes;
import java.util.List;

/**
 *
 * @author normenhansen
 */
public class CylinderCollisionShape extends CollisionShape{

    public CylinderCollisionShape(Node node) {
        createCollisionCylinder(node);
    }

    public CylinderCollisionShape(BoundingBox volume) {
        createCollisionCylinder(volume);
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
        createCollisionCylinder(volume);
    }

    private void createCollisionCylinder(BoundingBox volume){
        if(5==5)
            throw (new UnsupportedOperationException("Not implemented yet."));
        javax.vecmath.Vector3f halfExtents=new javax.vecmath.Vector3f(volume.xExtent,volume.yExtent,volume.zExtent);
        CylinderShape capShape=new CylinderShape(halfExtents);
        cShape=capShape;
        type=Shapes.CYLINDER;
    }

}
