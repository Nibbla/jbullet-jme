/*
 * Copyright (c) 2009-2010 Normen Hansen
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
package com.jmex.bullet.collision.shapes;

//import com.bulletphysics.collision.shapes.CapsuleShape;
import com.jme.bounding.BoundingCapsule;
import com.jme.math.FastMath;
import com.jme.scene.Node;
import com.jme.scene.Spatial;
import com.jmex.bullet.collision.shapes.CollisionShape.ShapeTypes;
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
        cShape=createCollisionCapsule(radius, height);
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
        cShape=createCollisionCapsule(radius, height);
        type=ShapeTypes.CAPSULE;
    }

    native long createCollisionCapsule(float radius, float height);
}
