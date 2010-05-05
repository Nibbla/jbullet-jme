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

import com.bulletphysics.collision.shapes.BoxShape;
import com.jme.bounding.BoundingBox;
import com.jme.math.Vector3f;
import com.jme.scene.Node;
import com.jme.scene.Spatial;
import com.jme.util.export.InputCapsule;
import com.jme.util.export.JMEExporter;
import com.jme.util.export.JMEImporter;
import com.jme.util.export.OutputCapsule;
import com.jmex.jbullet.util.Converter;

import java.io.IOException;
import java.util.List;

/**
 * Basic box collision shape
 * @author normenhansen
 */
public class BoxCollisionShape extends CollisionShape{
	public BoxCollisionShape() {
        
    }

    /**
     * creates a collision shape from the bounding volume of the given node
     * @param node the node to get the BoundingVolume from
     */
    public BoxCollisionShape(Node node) {
        createCollisionBox(node);
    }

    /**
     * creates a collision shape from the given bounding volume
     * @param volume the BoundingVolume to use
     */
    public BoxCollisionShape(BoundingBox volume) {
        createCollisionBox(volume);
    }

    /**
     * creates a collision box from the given halfExtents
     * @param halfExtents the halfExtents of the CollisionBox
     */
    public BoxCollisionShape(Vector3f halfExtents) {
        BoxShape sphere=new BoxShape(Converter.convert(halfExtents));
        cShape=sphere;
        type=ShapeTypes.BOX;
    }

    /**
     * creates a box in the physics space that represents this Node and all
     * children. The extents are computed from the world bound of this Node.
     */
    private void createCollisionBox(Node node) {
        List<Spatial> children=node.getChildren();
        if(children.size()==0){
            throw (new UnsupportedOperationException("PhysicsNode has no children, cannot compute collision box"));
        }
        if(!(node.getWorldBound() instanceof BoundingBox)){
            node.setModelBound(new BoundingBox());
        }
        node.updateModelBound();
        node.updateGeometricState(0,true);
        node.updateWorldBound();
        BoundingBox volume=(BoundingBox)node.getWorldBound();
        createCollisionBox(volume);
    }

    private void createCollisionBox(BoundingBox volume) {
        javax.vecmath.Vector3f halfExtents=new javax.vecmath.Vector3f(volume.xExtent - volume.getCenter().x,
                volume.yExtent - volume.getCenter().y,
                volume.zExtent - volume.getCenter().z);
        BoxShape sphere=new BoxShape(halfExtents);
        cShape=sphere;
        type=ShapeTypes.BOX;
    }

	@Override
	public Class getClassTag() {
		
		return BoxCollisionShape.class;
	}

	@Override
	public void read(JMEImporter im) throws IOException {
		InputCapsule capsule = im.getCapsule(this);
		
		Vector3f halfExtends = (Vector3f) capsule.readSavable("halfExtends", Vector3f.ZERO);
		 BoxShape sphere=new BoxShape(Converter.convert(halfExtends));
	     cShape=sphere;
	     type=ShapeTypes.BOX;
		
		
	}

	@Override
	public void write(JMEExporter ex) throws IOException {
		OutputCapsule capsule = ex.getCapsule( this );
		javax.vecmath.Vector3f halfExtends = new javax.vecmath.Vector3f();
		//@ With margin or without ?
		((BoxShape)cShape).getHalfExtentsWithMargin(halfExtends);
		capsule.write(Converter.convert(halfExtends), "halfExtends", Vector3f.ZERO);
		
	}

}
