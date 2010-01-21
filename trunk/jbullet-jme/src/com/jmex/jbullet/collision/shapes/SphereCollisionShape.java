/*
 * Copyright (c) 2009 Normen Hansen
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
     * creates a collision shape from the bounding volume of the given node
     * @param node the node to get the BoundingVolume from
     */
    public SphereCollisionShape(Node node) {
        createCollisionSphere(node);
    }

    /**
     * creates a collision shape from the given bounding volume
     * @param volume the BoundingVolume to use
     */
    public SphereCollisionShape(BoundingSphere volume) {
        createCollisionSphere(volume);
    }

    /**
     * creates a SphereCollisionShape with the given radius
     * @param radius
     */
    public SphereCollisionShape(float radius) {
        SphereShape sphere=new SphereShape(radius);
        cShape=sphere;
        type=ShapeTypes.SPHERE;
    }

    /**
     * creates a sphere in the physics space that represents this Node and all
     * children. The radius is computed from the world bound of this Node.
     */
    private void createCollisionSphere(Node node) {
        List<Spatial> children=node.getChildren();
        if(children.size()==0){
            throw (new UnsupportedOperationException("PhysicsNode has no children, cannot compute collision sphere"));
        }
        if(!(node.getWorldBound() instanceof BoundingSphere)){
            node.setModelBound(new BoundingSphere());
            node.updateGeometricState(0,true);
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
