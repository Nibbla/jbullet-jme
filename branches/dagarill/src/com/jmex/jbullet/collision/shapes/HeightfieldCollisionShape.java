/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.jmex.jbullet.collision.shapes;

import java.io.IOException;

import com.bulletphysics.dom.HeightfieldTerrainShape;
import com.jme.math.FastMath;
import com.jme.math.Vector3f;
import com.jme.scene.TriMesh;
import com.jme.util.export.InputCapsule;
import com.jme.util.export.JMEExporter;
import com.jme.util.export.JMEImporter;
import com.jme.util.export.OutputCapsule;
import com.jmex.jbullet.util.Converter;

/**
 * Uses Bullet Physics Heightfield terrain collision system. This is MUCH faster
 * than using a regular mesh.
 * There are a couple tricks though:
 *	-No rotation or translation is supported.
 *	-The collision bbox must be centered around 0,0,0 with the height above and below the y-axis being
 *	equal on either side. If not, the whole collision box is shifted vertically and things don't collide
 *	as they should.
 * 
 * @author Brent Owens
 */
public class HeightfieldCollisionShape extends CollisionShape {

	//protected HeightfieldTerrainShape heightfieldShape;
	protected int heightStickWidth;
	protected int heightStickLength;
	protected float[] heightfieldData;
	protected float heightScale;
	protected float minHeight;
	protected float maxHeight;
	protected int upAxis;
	protected boolean flipQuadEdges;

	public HeightfieldCollisionShape() {

	}

	public HeightfieldCollisionShape(float[] heightmap) {
		createCollisionHeightfield(heightmap, Vector3f.UNIT_XYZ);
	}

	public HeightfieldCollisionShape(float[] heightmap, Vector3f scale) {
		createCollisionHeightfield(heightmap, scale);
	}

	protected void createCollisionHeightfield(float[] heightmap, Vector3f worldScale) {
		this.scale = worldScale;
		this.heightScale = 1;//don't change away from 1, we use worldScale instead to scale
		
		this.heightfieldData = heightmap;

		float min = heightfieldData[0];
		float max = heightfieldData[0];
		// calculate min and max height
		for (int i=0; i<heightfieldData.length; i++) {
			if (heightfieldData[i] < min)
				min = heightfieldData[i];
			if (heightfieldData[i] > max)
				max = heightfieldData[i];
		}
		// we need to center the terrain collision box at 0,0,0 for BulletPhysics. And to do that we need to set the
		// min and max height to be equal on either side of the y axis, otherwise it gets shifted and collision is incorrect.
		if (max < 0)
			max = -min;
		else {
			if (Math.abs(max) > Math.abs(min))
				min = -max;
			else
				max = -min;
		}
		this.minHeight = min;
		this.maxHeight = max;

		this.upAxis = HeightfieldTerrainShape.YAXIS;
		this.flipQuadEdges = false;

		heightStickWidth = (int) FastMath.sqrt(heightfieldData.length);
		heightStickLength = heightStickWidth;


		createShape();
	}
	
	protected void createShape() {

		HeightfieldTerrainShape shape = new HeightfieldTerrainShape(heightStickWidth, heightStickLength, heightfieldData, heightScale, minHeight, maxHeight, upAxis, flipQuadEdges);
		shape.setLocalScaling(new javax.vecmath.Vector3f(scale.x, scale.y, scale.z));
		cShape = shape;
		cShape.setLocalScaling(Converter.convert(getScale()));
                cShape.setMargin(margin);
	}

	public TriMesh createJmeMesh(){
        //TODO return Converter.convert(bulletMesh);
		return null;
    }

    public void write(JMEExporter ex) throws IOException {
        super.write(ex);
        OutputCapsule capsule = ex.getCapsule(this);
        capsule.write(heightStickWidth, "heightStickWidth", 0);
        capsule.write(heightStickLength, "heightStickLength", 0);
        capsule.write(heightScale, "heightScale", 0);
        capsule.write(minHeight, "minHeight", 0);
        capsule.write(maxHeight, "maxHeight", 0);
        capsule.write(upAxis, "upAxis", 1);
        capsule.write(heightfieldData, "heightfieldData", new float[0]);
        capsule.write(flipQuadEdges, "flipQuadEdges", false);
    }

    public void read(JMEImporter im) throws IOException {
        super.read(im);
        InputCapsule capsule = im.getCapsule(this);
        heightStickWidth = capsule.readInt("heightStickWidth", 0);
        heightStickLength = capsule.readInt("heightStickLength", 0);
        heightScale = capsule.readFloat("heightScale", 0);
        minHeight = capsule.readFloat("minHeight", 0);
        maxHeight = capsule.readFloat("maxHeight", 0);
        upAxis = capsule.readInt("upAxis", 1);
        heightfieldData = capsule.readFloatArray("heightfieldData", new float[0]);
        flipQuadEdges = capsule.readBoolean("flipQuadEdges", false);
        createShape();
    }
    
    @Override
    public Class getClassTag() {
    	return getClass();
    }

}
