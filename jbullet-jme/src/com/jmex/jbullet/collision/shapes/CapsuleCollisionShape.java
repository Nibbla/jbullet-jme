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
import com.jmex.jbullet.collision.shapes.CollisionShape.Shapes;
import java.util.List;

/**
 *
 * @author normenhansen
 */
public class CapsuleCollisionShape extends CollisionShape{

    public CapsuleCollisionShape(Node node) {
        createCollisionCapsule(node);
    }

    public CapsuleCollisionShape(BoundingCapsule volume) {
        createCollisionCapsule(volume);
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
        type=Shapes.CAPSULE;
    }

}
