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
package com.jmex.jbullet.nodes;

import com.bulletphysics.collision.dispatch.CollisionFlags;
import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.dynamics.character.KinematicCharacterController;
import com.jme.math.Quaternion;
import com.jme.math.Vector3f;
import com.jme.scene.Spatial;
import com.jme.util.export.InputCapsule;
import com.jme.util.export.JMEExporter;
import com.jme.util.export.JMEImporter;
import com.jme.util.export.OutputCapsule;
import com.jmex.jbullet.collision.shapes.CollisionShape;
import com.jmex.jbullet.util.Converter;
import java.io.IOException;

/**
 *
 * @author normenhansen
 */
public class PhysicsCharacterNode extends PhysicsGhostNode{
	private KinematicCharacterController character;

    private float stepHeight;

    private Vector3f walkDirection=new Vector3f();


    public PhysicsCharacterNode(Spatial spat, int shapeType, float stepHeight) {
        super(spat, shapeType);
        this.stepHeight=stepHeight;
        if(shapeType==CollisionShape.ShapeTypes.MESH)
            throw (new UnsupportedOperationException("Kinematic character nodes cannot have meshes as collision shape"));
        character=new KinematicCharacterController(gObject, (ConvexShape)cShape.getCShape(), stepHeight);
    }

    public PhysicsCharacterNode(Spatial spat, CollisionShape shape, float stepHeight) {
        super(spat, shape);
        if(!(shape.getCShape() instanceof ConvexShape))
            throw (new UnsupportedOperationException("Kinematic character nodes can only have convex collision shapes"));
        this.stepHeight=stepHeight;
        character=new KinematicCharacterController(gObject, (ConvexShape)cShape.getCShape(), stepHeight);
    }

    protected Quaternion setWorldRotation( Quaternion rot ) {
        return rot;
    }


    @Override
    protected void buildObject() {
        super.buildObject();
        gObject.setCollisionFlags(CollisionFlags.CHARACTER_OBJECT);
    }

    public void warp(Vector3f location){
        character.warp(Converter.convert(location));
    }

    /**
     * set the walk direction, works continuously
     * @param vec the walk direction to set
     */
    public void setWalkDirection(Vector3f vec){
        walkDirection.set(vec);
        character.setWalkDirection(Converter.convert(walkDirection));
    }

    public void setUpAxis(int axis){
        character.setUpAxis(axis);
    }

    public void setMaxJumpHeight(float height){
        character.setMaxJumpHeight(height);
    }

    public void jump() {
        character.jump();
    }

    /**
     * used internally
     */
    public KinematicCharacterController getCharacterController() {
        return character;
    }

    @Override
    public void destroy() {
        super.destroy();
    }

      @Override
    public void write(JMEExporter e) throws IOException {
    	super.write(e);
        OutputCapsule capsule = e.getCapsule( this );
        capsule.write(stepHeight, "stepHeight", 0);
        capsule.write(walkDirection, "walkDirection", Vector3f.ZERO);
      }

      @Override
	public void read(JMEImporter e) throws IOException {
            super.read(e);
            gObject.setCollisionFlags(CollisionFlags.CHARACTER_OBJECT);
            InputCapsule capsule = e.getCapsule(this);
            stepHeight= capsule.readFloat("stepHeight", 0);
            walkDirection = (Vector3f) capsule.readSavable("walkDirection", Vector3f.ZERO);
            character=new KinematicCharacterController(gObject, (ConvexShape)cShape.getCShape(), stepHeight);
      }

       @Override
	public Class getClassTag() {

		return PhysicsGhostNode.class;
	}

}
