/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.jmex.jbullet.collision.shapes;

import com.bulletphysics.collision.shapes.CapsuleShape;
import com.jme.bounding.BoundingCapsule;
import com.jme.math.FastMath;
import com.jme.scene.Node;
import com.jme.scene.Spatial;
import com.jmex.jbullet.collision.shapes.CollisionShape.ShapeTypes;
import java.util.List;

/**
 * Basic capsule collision shape
 * @author normenhansen
 */
public class CapsuleCollisionShape extends CollisionShape{

    /**
     * Creates a collision shape from the bounding volume of the given node
     * @param node the node to get the BoundingVolume from
     */
    public CapsuleCollisionShape(Node node) {
        createCollisionCapsule(node);
    }

    /**
     * Creates a collision shape from the given bounding volume
     * @param volume the BoundingVolume to use
     */
    public CapsuleCollisionShape(BoundingCapsule volume) {
        createCollisionCapsule(volume);
    }

    /**
     * Creates a new CapsuleCollisionShape with the given radius and height
     * @param radius
     * @param height
     */
    public CapsuleCollisionShape(float radius, float height) {
        CapsuleShape capShape=new CapsuleShape(radius,height);
        cShape=capShape;
        type=ShapeTypes.CAPSULE;
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
        createCollisionCapsule(capsule);
    }

    private void createCollisionCapsule(BoundingCapsule capsule) {
        float radius=capsule.getRadius();
        float volume=capsule.getVolume();
        volume-= ( ((4.0f/3.0f) * FastMath.PI ) * FastMath.pow(radius,3) );
        float height=(volume/(FastMath.PI*FastMath.pow(radius,2)));
        height+=(radius*2);
        CapsuleShape capShape=new CapsuleShape(capsule.getRadius(),height);
        cShape=capShape;
        type=ShapeTypes.CAPSULE;
    }

}
