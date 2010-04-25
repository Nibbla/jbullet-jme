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
package jmetest.jbullet;


import java.util.concurrent.Callable;

import com.jme.math.Vector3f;
import com.jme.scene.shape.Box;
import com.jme.scene.shape.Capsule;
import com.jme.scene.shape.Cylinder;
import com.jme.scene.shape.Sphere;
import com.jme.util.GameTaskQueueManager;
import com.jmex.editors.swing.settings.GameSettingsPanel;
import com.jmex.game.StandardGame;
import com.jmex.game.state.DebugGameState;
import com.jmex.game.state.GameStateManager;
import com.jmex.jbullet.PhysicsSpace;
import com.jmex.jbullet.collision.shapes.CollisionShape;
import com.jmex.jbullet.nodes.PhysicsNode;

/**
 * This is a basic Test of jbullet-jme functions
 *
 * @author normenhansen
 */
public class TestSimplePhysics {

    public static void setupGame(){
        // creates and initializes the PhysicsSpace
        final PhysicsSpace pSpace=PhysicsSpace.getPhysicsSpace();

        // Create a DebugGameState
        // - override the update method to update/sync physics space
        DebugGameState state = new DebugGameState(){

            @Override
            public void update(float tpf) {
                pSpace.update(tpf);
                super.update(tpf);
            }
            
        };

        // Add a physics sphere to the world
        Sphere sphere=new Sphere("physicssphere",16,16,1f);
        PhysicsNode physicsSphere=new PhysicsNode(sphere,CollisionShape.ShapeTypes.SPHERE);
        physicsSphere.setLocalTranslation(new Vector3f(3,6,0));
        state.getRootNode().attachChild(physicsSphere);
        physicsSphere.updateRenderState();
        pSpace.add(physicsSphere);

        // Add a physics sphere to the world using the collision shape from sphere one
        Sphere sphere2=new Sphere("physicssphere",16,16,1f);
        PhysicsNode physicsSphere2=new PhysicsNode(sphere2,physicsSphere.getCollisionShape());
        physicsSphere2.setLocalTranslation(new Vector3f(4,8,0));
        state.getRootNode().attachChild(physicsSphere2);
        physicsSphere2.updateRenderState();
        pSpace.add(physicsSphere2);

        // Add a physics box to the world
        Box boxGeom=new Box("physicsbox",Vector3f.ZERO,1f,1f,1f);
        PhysicsNode physicsBox=new PhysicsNode(boxGeom,CollisionShape.ShapeTypes.BOX);
        physicsBox.setFriction(0.1f);
        physicsBox.setLocalTranslation(new Vector3f(.6f,4,.5f));
        state.getRootNode().attachChild(physicsBox);
        physicsBox.updateRenderState();
        pSpace.add(physicsBox);

        Cylinder cylGeom=new Cylinder("physicscyliner",16,16,1f,3f);
        PhysicsNode physicsCylinder=new PhysicsNode(cylGeom, CollisionShape.ShapeTypes.CYLINDER);
        physicsCylinder.setLocalTranslation(new Vector3f(-5,4,0));
        state.getRootNode().attachChild(physicsCylinder);
        physicsCylinder.updateRenderState();
        pSpace.add(physicsCylinder);

        Capsule capGeom=new Capsule("physicscapsule",16,16,16,0.5f,2f);
        PhysicsNode physicsCapsule=new PhysicsNode(capGeom, CollisionShape.ShapeTypes.CAPSULE);
        physicsCapsule.setFriction(0.1f);
        physicsCapsule.setLocalTranslation(new Vector3f(-8,4,0));
        state.getRootNode().attachChild(physicsCapsule);
        physicsCapsule.updateRenderState();
        pSpace.add(physicsCapsule);
//        physicsCapsule.setMass(10f);

        // Join the physics objects with a Point2Point joint
//        PhysicsPoint2PointJoint joint=new PhysicsPoint2PointJoint(physicsSphere, physicsBox, new Vector3f(-2,0,0), new Vector3f(2,0,0));
//        PhysicsHingeJoint joint=new PhysicsHingeJoint(physicsSphere, physicsBox, new Vector3f(-2,0,0), new Vector3f(2,0,0), Vector3f.UNIT_Z,Vector3f.UNIT_Z);
//        pSpace.add(joint);
        
        // an obstacle mesh, does not move (mass=0)
        PhysicsNode node2=new PhysicsNode(new Sphere("physicsobstaclemesh",16,16,1.2f),CollisionShape.ShapeTypes.MESH,0);
        node2.setLocalTranslation(new Vector3f(2.5f,-4,0f));
        state.getRootNode().attachChild(node2);
        node2.updateRenderState();
        pSpace.add(node2);

        // the floor, does not move (mass=0)
        PhysicsNode node3=new PhysicsNode(new Box("physicsfloor",Vector3f.ZERO,100f,0.2f,100f),CollisionShape.ShapeTypes.MESH,0);
        node3.setLocalTranslation(new Vector3f(0f,-6,0f));
        state.getRootNode().attachChild(node3);
        node3.updateRenderState();
        pSpace.add(node3);

        
        // Add the gamestate to the manager
        GameStateManager.getInstance().attachChild(state);
        // Activate the game state
        state.setActive(true);

    }

	public static void main(String[] args) throws Exception {
	    // Enable statistics gathering
	    System.setProperty("jme.stats", "set");

		// Instantiate StandardGame
		StandardGame game = new StandardGame("A Simple Test");
		// Show settings screen
		if (GameSettingsPanel.prompt(game.getSettings())) {
			// Start StandardGame, it will block until it has initialized successfully, then return
			game.start();

			GameTaskQueueManager.getManager().update(new Callable<Void>() {

				public Void call() throws Exception {
                    setupGame();
					return null;
				}
			});
		}
	}
}
