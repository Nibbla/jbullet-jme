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

import com.bulletphysics.collision.shapes.CylinderShape;
import com.bulletphysics.collision.shapes.CylinderShapeX;
import com.bulletphysics.collision.shapes.CylinderShapeZ;
import com.g3d.bounding.BoundingBox;
import com.g3d.math.Vector3f;
import com.g3d.scene.Node;
import com.g3d.scene.Spatial;
import com.jmex.jbullet.collision.shapes.CollisionShape.ShapeTypes;
import com.jmex.jbullet.util.Converter;
import java.util.List;

/**
 * Basic cylinder collision shape
 * @author normenhansen
 */
public class CylinderCollisionShape extends CollisionShape{

    /**
     * creates a collision shape from the bounding volume of the given node
     * @param node the node to get the BoundingVolume from
     */
    public CylinderCollisionShape(Node node) {
        createCollisionCylinder(node);
    }

    /**
     * creates a collision shape from the given bounding volume
     * @param volume the BoundingVolume to use
     */
    public CylinderCollisionShape(BoundingBox volume) {
        createCollisionCylinder(volume);
    }

    /**
     * creates a cylinder shape from the given halfextents
     * @param halfExtents the halfextents to use
     */
    public CylinderCollisionShape(Vector3f halfExtents) {
        CylinderShape capShape=new CylinderShapeZ(Converter.convert(halfExtents));
        cShape=capShape;
        type=ShapeTypes.CYLINDER;
    }

    /**
     * Creates a cylinder shape around the given axis from the given halfextents
     * @param halfExtents the halfextents to use
     * @param axis (0=X,1=Y,2=Z)
     */
    public CylinderCollisionShape(Vector3f halfExtents, int axis) {
        switch(axis){
            case 0:
                cShape=new CylinderShapeX(Converter.convert(halfExtents));
            break;
            case 1:
                cShape=new CylinderShape(Converter.convert(halfExtents));
            break;
            case 2:
                cShape=new CylinderShapeZ(Converter.convert(halfExtents));
            break;
        }
        type=ShapeTypes.CYLINDER;
    }

    private void createCollisionCylinder(Node node){
        List<Spatial> children=node.getChildren();
        if(children.size()==0){
            throw (new UnsupportedOperationException("PhysicsNode has no children, cannot compute collision cylinder"));
        }
        node.setModelBound(new BoundingBox());
        node.updateModelBound();
        node.updateGeometricState();
//        node.updateWorldBound();
        BoundingBox volume=(BoundingBox)node.getWorldBound();
        createCollisionCylinder(volume);
    }

    private void createCollisionCylinder(BoundingBox volume){
        javax.vecmath.Vector3f halfExtents=new javax.vecmath.Vector3f(
                volume.getXExtent() - volume.getCenter().x,
                volume.getYExtent() - volume.getCenter().y,
                volume.getZExtent() - volume.getCenter().z);
        CylinderShapeZ capShape=new CylinderShapeZ(halfExtents);
        cShape=capShape;
        type=ShapeTypes.CYLINDER;
    }

}
