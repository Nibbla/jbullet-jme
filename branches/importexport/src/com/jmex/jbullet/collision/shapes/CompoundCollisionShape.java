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

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import com.bulletphysics.collision.shapes.BoxShape;
import com.bulletphysics.collision.shapes.CapsuleShape;
import com.bulletphysics.collision.shapes.CompoundShape;
import com.bulletphysics.collision.shapes.CompoundShapeChild;
import com.bulletphysics.collision.shapes.CylinderShape;
import com.bulletphysics.collision.shapes.CylinderShapeX;
import com.bulletphysics.collision.shapes.SphereShape;
import com.bulletphysics.linearmath.Transform;
import com.jme.math.Matrix3f;
import com.jme.math.Vector3f;
import com.jme.util.export.InputCapsule;
import com.jme.util.export.JMEExporter;
import com.jme.util.export.JMEImporter;
import com.jme.util.export.OutputCapsule;
import com.jmex.jbullet.util.CompoundCollisionShapeChild;
import com.jmex.jbullet.util.Converter;

/**
 * A CompoundCollisionShape allows combining multiple base shapes
 * to generate a more sophisticated shape.
 * @author normenhansen
 */
public class CompoundCollisionShape extends CollisionShape{

    public CompoundCollisionShape() {
        cShape=new CompoundShape();
        this.type=CollisionShape.ShapeTypes.COMPOUND;
    }

    /**
     * adds a child shape at the given local translation
     * @param shape the child shape to add
     * @param location the local location of the child shape
     */
    public void addChildShape(CollisionShape shape, Vector3f location){
        Transform transA=new Transform(Converter.convert(new Matrix3f()));
        Converter.convert(location,transA.origin);
        ((CompoundShape)cShape).addChildShape(transA, shape.getCShape());
    }

    /**
     * removes a child shape
     * @param shape the child shape to remove
     */
    public void removeChildShape(CollisionShape shape){
        ((CompoundShape)cShape).removeChildShape(shape.getCShape());
    }

	@Override
	public Class getClassTag() {
		// TODO Auto-generated method stub
		return this.getClass();
	}

	@Override
	public void read(JMEImporter im) throws IOException {
		InputCapsule capsule = im.getCapsule(this);
		
		ArrayList<CompoundCollisionShapeChild> childs = capsule.readSavableArrayList("childs", null);
		for (int i = 0; i < childs.size(); i++) {
			
			 ((CompoundShape)cShape).addChildShape(childs.get(0).getTransform().convert(), childs.get(i).getShape().getCShape());
		}
		
		//throw (new UnsupportedOperationException("Not implemented yet."));
	}
	
	private CollisionShape transform(com.bulletphysics.collision.shapes.CollisionShape shape){
		if(shape instanceof BoxShape){
			return new BoxCollisionShape((BoxShape) shape);
		}
		if(shape instanceof SphereShape){
			return new SphereCollisionShape((SphereShape) shape);
		}
		if(shape instanceof CylinderShape){
			return new CylinderCollisionShape((CylinderShape) shape);
		}
		if(shape instanceof CapsuleShape){
			return new CapsuleCollisionShape((CapsuleShape) shape);
		}
	
		return null;
	}
	
	

	@Override
	public void write(JMEExporter ex) throws IOException {
		OutputCapsule capsule = ex.getCapsule( this );
		CompoundShape cshape = ((CompoundShape)cShape);
		List<CompoundShapeChild> list= cshape.getChildList();
		ArrayList<CompoundCollisionShapeChild> childs = new ArrayList<CompoundCollisionShapeChild>();
		for (int i = 0; i < list.size(); i++) {
			CompoundShapeChild csc = list.get(i);
			CollisionShape shape = transform(csc.childShape);
			com.jmex.jbullet.util.Transform transform = new com.jmex.jbullet.util.Transform(csc.transform);
			CompoundCollisionShapeChild child = new CompoundCollisionShapeChild(shape, transform);
			childs.add(child);
		}
		
		capsule.writeSavableArrayList(childs, "childs", null);
		
		//throw (new UnsupportedOperationException("Not implemented yet."));
	}

}
