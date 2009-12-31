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
import com.jme.math.Vector3f;
import com.jme.scene.Spatial;
import com.jmex.jbullet.collision.CollisionShape;
import com.jmex.jbullet.util.Converter;
import java.util.concurrent.Callable;

/**
 *
 * @author normenhansen
 */
public class PhysicsCharacterNode extends PhysicsGhostNode{
	private KinematicCharacterController character;

    private float stepHeight;
    private float fallSpeed;
    private float jumpSpeed;

    private Vector3f walkDirection=new Vector3f();

    private int upAxis=0;
    private float maxJumpHeight=1.0f;

    public PhysicsCharacterNode(Spatial spat, int shapeType, float stepHeight) {
        super(spat, shapeType);
        this.stepHeight=stepHeight;
        if(shapeType==CollisionShape.Shapes.MESH)
            throw (new UnsupportedOperationException("Kinematic character nodes cannot have meshes as collision shape"));
        character=new KinematicCharacterController(gObject, (ConvexShape)cShape.getCShape(), stepHeight);
    }

    @Override
    protected void buildObject() {
        super.buildObject();
        gObject.setCollisionFlags(CollisionFlags.CHARACTER_OBJECT);
    }

    public void setWalkDirection(Vector3f vec){
        walkDirection.set(vec);
        pQueue.enqueue(doApplyWalkDirection);
    }

    private Callable doApplyWalkDirection=new Callable(){
        public Object call() throws Exception {
            character.setWalkDirection(Converter.convert(walkDirection));
            return null;
        }
    };

    public void setUpAxis(int axis){
        upAxis=axis;
        pQueue.enqueue(doApplyUpAxis);
    }

    private Callable doApplyUpAxis=new Callable(){
        public Object call() throws Exception {
            character.setWalkDirection(Converter.convert(walkDirection));
            return null;
        }
    };

    public void setMaxJumpHeight(float height){
        maxJumpHeight=height;
        pQueue.enqueue(doApplyMaxJumpHeight);
    }

    private Callable doApplyMaxJumpHeight=new Callable(){
        public Object call() throws Exception {
            character.setMaxJumpHeight(maxJumpHeight);
            return null;
        }
    };

    public void jump() {
        pQueue.enqueue(doApplyJump);
    }

    private Callable doApplyJump=new Callable(){
        public Object call() throws Exception {
            character.jump();
            return null;
        }
    };

    /**
     * @return the character
     */
    public KinematicCharacterController getCharacterController() {
        return character;
    }

}
