package com.jmex.jbullet.util;

import java.io.IOException;


import com.jme.math.Matrix3f;
import com.jme.math.Vector3f;
import com.jme.util.export.InputCapsule;
import com.jme.util.export.JMEExporter;
import com.jme.util.export.JMEImporter;
import com.jme.util.export.OutputCapsule;
import com.jme.util.export.Savable;

public class Transform implements Savable {
	
	public Transform(){
		
	}
	
	
	public Transform(Matrix3f rot, Vector3f loc){
		basis=rot;
		origin= loc;
	}
	
	public Transform(com.bulletphysics.linearmath.Transform t){
		basis=Converter.convert(t.basis);
		origin= Converter.convert(t.origin);
	}
	
	public com.bulletphysics.linearmath.Transform convert(){
		
		com.bulletphysics.linearmath.Transform  transform = new com.bulletphysics.linearmath.Transform (Converter.convert(basis));
		Converter.convert(origin,transform.origin);
		return transform;
		
	}
	

	public Matrix3f basis = new Matrix3f();
	
	/** Translation vector of this Transform. */
	public Vector3f origin = new Vector3f();
	
	@Override
	public Class getClassTag() {
		return this.getClass();
	}

	@Override
	public void read(JMEImporter im) throws IOException {
		InputCapsule capsule = im.getCapsule(this);
		basis = (Matrix3f) capsule.readSavable("rot", null);
		
		origin= (Vector3f) capsule.readSavable("loc", null);
	}

	@Override
	public void write(JMEExporter ex) throws IOException {
		OutputCapsule capsule = ex.getCapsule( this );
		capsule.write(basis, "rot", null);
		capsule.write(origin, "loc", null);

	}

}
