package com.jmex.jbullet.util;

import java.io.IOException;

import com.jme.math.Vector3f;
import com.jme.util.export.InputCapsule;
import com.jme.util.export.JMEExporter;
import com.jme.util.export.JMEImporter;
import com.jme.util.export.OutputCapsule;
import com.jme.util.export.Savable;
import com.jmex.jbullet.collision.shapes.CollisionShape;

public class CompoundCollisionShapeChild implements Savable {

	private CollisionShape shape;
	private Transform transform;
	
	public CompoundCollisionShapeChild(){
		
	}
	
	public CompoundCollisionShapeChild(CollisionShape shape, Transform transform){
		this.shape=shape;
		this.transform=transform;
	}
	
	

	

	@Override
	public Class getClassTag() {
	
		return this.getClass();
	}

	@Override
	public void read(JMEImporter im) throws IOException {
		InputCapsule capsule = im.getCapsule(this);
		shape =(CollisionShape) capsule.readSavable("shape", null);
		transform =(Transform) capsule.readSavable("transform", null);
	}

	public CollisionShape getShape() {
		return shape;
	}

	public void setShape(CollisionShape shape) {
		this.shape = shape;
	}

	public Transform getTransform() {
		return transform;
	}

	public void setTransform(Transform transform) {
		this.transform = transform;
	}

	@Override
	public void write(JMEExporter ex) throws IOException {
		OutputCapsule capsule = ex.getCapsule( this );
		capsule.write(shape, "shape", null);
		capsule.write(transform, "transform", null);
	}

}
