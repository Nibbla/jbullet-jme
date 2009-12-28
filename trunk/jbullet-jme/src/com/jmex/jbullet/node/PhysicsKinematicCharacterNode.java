/*
 * Copyright (c) 2003-2009 jMonkeyEngine
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
 * * Neither the name of 'jMonkeyEngine' nor the names of its contributors
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
package com.jmex.jbullet.node;

import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.dynamics.character.KinematicCharacterController;
import com.jme.math.Vector3f;
import com.jme.scene.Spatial;
import com.jmex.jbullet.collision.CollisionShape;
import com.jmex.jbullet.util.Converter;

/**
 *
 * @author normenhansen
 */
public class PhysicsKinematicCharacterNode extends PhysicsGhostNode{
	protected KinematicCharacterController character;

    private float fallSpeed;
    private float jumpSpeed;

    private Vector3f walkDirection=new Vector3f();

    private int upAxis=0;
    private float maxJumpHeight=1.0f;

    private boolean applySpeeds=false;
    private boolean applyJump=false;
    private boolean applyJumpHeight=false;
    private boolean applyDirection=false;
    private boolean applyAxis=false;

    public PhysicsKinematicCharacterNode(Spatial spat, int shapeType, float stepHeight) {
        super(spat, shapeType);
        if(shapeType==CollisionShape.Shapes.MESH)
            throw (new UnsupportedOperationException("Kinematic character nodes cannot have meshes as collision shape"));
        character=new KinematicCharacterController(gObject, (ConvexShape)cShape.getCShape(), stepHeight);
    }

    public void setWalkDirection(Vector3f vec){
        walkDirection.set(vec);
        applyDirection=true;
    }

    public void setUpAxis(int axis){
        upAxis=axis;
        applyAxis=true;
    }


    public void setMaxJumpHeight(float height){
        maxJumpHeight=height;
        applyJumpHeight=true;
    }

    public void jump() {
        applyJump=true;
    }

    @Override
    public void syncPhysics() {
        super.syncPhysics();
        if(applySpeeds){
            character.setFallSpeed(fallSpeed);
            character.setJumpSpeed(jumpSpeed);
        }
        if(applyJumpHeight){
            character.setMaxJumpHeight(maxJumpHeight);
        }
        if(applyAxis){
            character.setUpAxis(upAxis);
            applyAxis=false;
        }
        if(applyJump){
            character.jump();
            applyJump=false;
        }
        if(applyDirection){
            //TODO: reuse vector?
            character.setWalkDirection(Converter.convert(walkDirection));
            applyDirection=false;
        }
    }

}
