/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package com.jmex.jbullet.collision.shapes.infos;

import java.io.IOException;

import com.jme.math.Matrix3f;
import com.jme.math.Vector3f;
import com.jme.util.export.InputCapsule;
import com.jme.util.export.JMEExporter;
import com.jme.util.export.JMEImporter;
import com.jme.util.export.OutputCapsule;
import com.jme.util.export.Savable;
import com.jmex.jbullet.collision.shapes.BoxCollisionShape;
import com.jmex.jbullet.collision.shapes.CollisionShape;

/**
 *
 * @author normenhansen
 */
public class ChildCollisionShape implements Savable {

    public Vector3f location;
    public Matrix3f rotation;
    public CollisionShape shape;

    public ChildCollisionShape() {
    }

    public ChildCollisionShape(Vector3f location, Matrix3f rotation, CollisionShape shape) {
        this.location = location;
        this.rotation = rotation;
        this.shape = shape;
    }

    public void write(JMEExporter ex) throws IOException {
        OutputCapsule capsule = ex.getCapsule(this);
        capsule.write(location, "location", new Vector3f());
        capsule.write(rotation, "rotation", new Matrix3f());
        capsule.write(shape, "shape", new BoxCollisionShape(new Vector3f(1, 1, 1)));
    }

    public void read(JMEImporter im) throws IOException {
        InputCapsule capsule = im.getCapsule(this);
        location = (Vector3f) capsule.readSavable("location", new Vector3f());
        rotation = (Matrix3f) capsule.readSavable("rotation", new Matrix3f());
        shape = (CollisionShape) capsule.readSavable("shape", new BoxCollisionShape(new Vector3f(1, 1, 1)));
    }
    
    @Override
    public Class getClassTag() {
    	return getClass();
    }
}
