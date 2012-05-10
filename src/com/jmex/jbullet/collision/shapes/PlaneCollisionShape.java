/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.jmex.jbullet.collision.shapes;

import java.io.IOException;

import com.bulletphysics.collision.shapes.StaticPlaneShape;
import com.jme.math.Plane;
import com.jme.util.export.InputCapsule;
import com.jme.util.export.JMEExporter;
import com.jme.util.export.JMEImporter;
import com.jme.util.export.OutputCapsule;
import com.jmex.jbullet.util.Converter;

/**
 *
 * @author normenhansen
 */
public class PlaneCollisionShape extends CollisionShape{
    private Plane plane;

    public PlaneCollisionShape() {
    }

    /**
     * Creates a plane Collision shape
     * @param plane the plane that defines the shape
     */
    public PlaneCollisionShape(Plane plane) {
        this.plane = plane;
        createShape();
    }

    public final Plane getPlane() {
        return plane;
    }

    public void write(JMEExporter ex) throws IOException {
        super.write(ex);
        OutputCapsule capsule = ex.getCapsule(this);
        capsule.write(plane, "collisionPlane", new Plane());
    }

    public void read(JMEImporter im) throws IOException {
        super.read(im);
        InputCapsule capsule = im.getCapsule(this);
        plane = (Plane) capsule.readSavable("collisionPlane", new Plane());
        createShape();
    }

    protected void createShape() {
        cShape = new StaticPlaneShape(Converter.convert(plane.getNormal()),plane.getConstant());
        cShape.setLocalScaling(Converter.convert(getScale()));
        cShape.setMargin(margin);
    }
    
    @Override
    public Class getClassTag() {
    	return null;
    }

}
