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

import com.bulletphysics.collision.shapes.CapsuleShape;
import com.bulletphysics.collision.shapes.CapsuleShapeX;
import com.bulletphysics.collision.shapes.CapsuleShapeZ;
import com.jme.bounding.BoundingCapsule;
import com.jme.math.FastMath;
import com.jme.scene.Node;
import com.jme.scene.Spatial;
import com.jme.util.export.InputCapsule;
import com.jme.util.export.JMEExporter;
import com.jme.util.export.JMEImporter;
import com.jme.util.export.OutputCapsule;
import com.jmex.jbullet.collision.shapes.CollisionShape.ShapeTypes;

import java.io.IOException;
import java.util.List;

/**
 * Basic capsule collision shape
 * @author normenhansen
 */
public class CapsuleCollisionShape extends CollisionShape{
	public CapsuleCollisionShape() {
		
    }
	
	public CapsuleCollisionShape(CapsuleShape shape) {
		cShape=shape;
	       type = ShapeTypes.CYLINDER;
    }
    /**
     * creates a collision shape from the bounding volume of the given node
     * @param node the node to get the BoundingVolume from
     */
    public CapsuleCollisionShape(Node node) {
        createCollisionCapsule(node);
    }

    /**
     * creates a collision shape from the given bounding volume
     * @param volume the BoundingVolume to use
     */
    public CapsuleCollisionShape(BoundingCapsule volume) {
        createCollisionCapsule(volume);
    }

    /**
     * creates a new CapsuleCollisionShape with the given radius and height
     * @param radius the radius of the capsule
     * @param height the height of the capsule
     */
    public CapsuleCollisionShape(float radius, float height) {
        CapsuleShape capShape=new CapsuleShape(radius,height);
        cShape=capShape;
        
        type=ShapeTypes.CAPSULE;
    }

    /**
     * creates a capsule shape around the given axis (0=X,1=Y,2=Z)
     * @param radius
     * @param height
     * @param axis
     */
    public CapsuleCollisionShape(float radius, float height, int axis) {
        switch(axis){
            case 0:
                cShape=new CapsuleShapeX(radius,height);
            break;
            case 1:
                cShape=new CapsuleShape(radius,height);
            break;
            case 2:
                cShape=new CapsuleShapeZ(radius,height);
            break;
        }
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
            node.updateGeometricState(0,true);
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

	@Override
	public Class getClassTag() {
		// TODO Auto-generated method stub
		return CapsuleCollisionShape.class;
	}

	@Override
	public void read(JMEImporter im) throws IOException {
		InputCapsule capsule = im.getCapsule(this);
		float height = capsule.readFloat("height", 0);
		float radius = capsule.readFloat("radius", 0);
		int axis = capsule.readInt("axis", 1);
		 switch(axis){
         case 0:
             cShape=new CapsuleShapeX(radius,height);
         break;
         case 1:
             cShape=new CapsuleShape(radius,height);
         break;
         case 2:
             cShape=new CapsuleShapeZ(radius,height);
         break;
     }
     type=ShapeTypes.CAPSULE;
	}

	@Override
	public void write(JMEExporter ex) throws IOException {
		OutputCapsule capsule = ex.getCapsule( this );
		
		if(cShape instanceof CapsuleShapeX ){
			capsule.write(0, "axis", 1);
		}else if (cShape instanceof CapsuleShapeZ){
			capsule.write(2, "axis", 1);
		}else{ // CapsuleShape
			capsule.write(1, "axis", 1);
		}
		//TODO halfheight x2 ?
		capsule.write(((CapsuleShape)cShape).getHalfHeight(), "height", 0);
		capsule.write(((CapsuleShape)cShape).getRadius(), "radius", 0);
		
	}

}
