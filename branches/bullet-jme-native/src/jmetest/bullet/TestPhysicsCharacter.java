/*
 * Copyright (c) 2009-2010 Normen Hansen
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
package jmetest.bullet;


import com.jme.input.KeyBindingManager;
import com.jme.input.KeyInput;
import java.util.concurrent.Callable;

import com.jme.math.Vector3f;
import com.jme.scene.shape.Box;
import com.jme.scene.shape.Sphere;
import com.jme.util.GameTaskQueueManager;
import com.jmex.editors.swing.settings.GameSettingsPanel;
import com.jmex.game.StandardGame;
import com.jmex.game.state.DebugGameState;
import com.jmex.game.state.GameStateManager;
import com.jmex.bullet.PhysicsSpace;
import com.jmex.bullet.collision.shapes.CollisionShape;
import com.jmex.bullet.nodes.PhysicsCharacterNode;
import com.jmex.bullet.nodes.PhysicsNode;

/**
 * This is a basic Test of jbullet-jme character nodes
 *
 * @author normenhansen
 */
public class TestPhysicsCharacter {
    private static PhysicsCharacterNode character;

    public static void setupGame(){
        // creates and initializes the PhysicsSpace
        final PhysicsSpace pSpace=PhysicsSpace.getPhysicsSpace(PhysicsSpace.BroadphaseTypes.AXIS_SWEEP_3);

        // add some keybindings to control the vehicle
        KeyBindingManager.getKeyBindingManager().set("key_accelerate",
                KeyInput.KEY_U);
        KeyBindingManager.getKeyBindingManager().set("key_brake",
                KeyInput.KEY_J);
        KeyBindingManager.getKeyBindingManager().set("key_steer_left",
                KeyInput.KEY_H);
        KeyBindingManager.getKeyBindingManager().set("key_steer_right",
                KeyInput.KEY_K);
        KeyBindingManager.getKeyBindingManager().set("key_action",
                KeyInput.KEY_SPACE);

        // Create a DebugGameState
        // - override the update method to update/sync physics space
        DebugGameState state = new DebugGameState(){
            CollisionShape shape;

            private boolean stoppedBrake=false;
            private boolean stoppedAccel=false;
            private boolean stoppedSteerL=false;
            private boolean stoppedSteerR=false;

            private boolean appliedBrake=false;
            private boolean appliedAccel=false;
            private boolean appliedSteerL=false;
            private boolean appliedSteerR=false;

            @Override
            public void update(float tpf) {
                pSpace.update(tpf);
                super.update(tpf);
                if (KeyBindingManager.getKeyBindingManager().isValidCommand(
                        "key_accelerate", true)) {
                    if(!appliedAccel){
                        character.setWalkDirection(Vector3f.UNIT_Z.mult(-.1f));
                        appliedAccel=true;
                        stoppedBrake=false;
                        stoppedAccel=false;
                    }
                }
                else if (KeyBindingManager.getKeyBindingManager().isValidCommand(
                        "key_brake", true)) {
                    if(!appliedBrake){
                        character.setWalkDirection(Vector3f.UNIT_Z.mult(.1f));
                        appliedBrake=true;
                        stoppedBrake=false;
                        stoppedAccel=false;
                    }
                }
                else if (KeyBindingManager.getKeyBindingManager().isValidCommand(
                        "key_steer_left", true)) {
                    if(!appliedSteerL){
                        character.setWalkDirection(Vector3f.UNIT_X.mult(-.1f));
                        stoppedSteerL=false;
                        appliedSteerL=true;
                    }
                }
                else if (KeyBindingManager.getKeyBindingManager().isValidCommand(
                        "key_steer_right", true)) {
                    if(!appliedSteerR){
                        character.setWalkDirection(Vector3f.UNIT_X.mult(.1f));
                        stoppedSteerR=false;
                        appliedSteerR=true;
                    }
                }
                else if(!stoppedSteerL||!stoppedSteerR||!stoppedAccel||!stoppedBrake){
                    character.setWalkDirection(Vector3f.ZERO);
                    stoppedSteerL=true;
                    appliedSteerL=false;
                    stoppedSteerR=true;
                    appliedSteerR=false;
                    stoppedBrake=true;
                    appliedBrake=false;
                    appliedAccel=false;
                    stoppedAccel=true;
                }

                if (KeyBindingManager.getKeyBindingManager().isValidCommand(
                        "key_action", false)) {
//                    character.setWalkDirection(Vector3f.ZERO);
                    character.jump();
                }

            }
            
        };
        state.setText("u,h,j,k = move character / space = jump");

        Sphere caps=new Sphere("character",8,8,2f);
        character=new PhysicsCharacterNode(caps, CollisionShape.ShapeTypes.SPHERE, .1f);
        character.setLocalTranslation(0,3,0);
        state.getRootNode().attachChild(character);
        character.updateRenderState();
        pSpace.add(character);

        Box box=new Box("physicsobstaclemesh",Vector3f.ZERO,.5f,.5f,.5f);
        PhysicsNode boxNode=new PhysicsNode(box, CollisionShape.ShapeTypes.BOX);
        boxNode.setLocalTranslation(6,-1,0);
        state.getRootNode().attachChild(boxNode);
        boxNode.updateRenderState();
        pSpace.add(boxNode);

        // an obstacle mesh, does not move (mass=0)
        PhysicsNode node2=new PhysicsNode(new Box("physicsobstaclemesh",Vector3f.ZERO,2,2,2),CollisionShape.ShapeTypes.BOX,0);
        node2.setLocalTranslation(new Vector3f(0f,-4,0f));
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
