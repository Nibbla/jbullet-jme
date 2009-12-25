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
package jmetest.jbullet;


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
import com.jmex.jbullet.PhysicsSpace;
import com.jmex.jbullet.collision.CollisionShape;
import com.jmex.jbullet.node.PhysicsVehicleNode;
import com.jmex.jbullet.node.PhysicsNode;

/**
 * This is a basic Test of jbullet-jme vehicles
 *
 * @author normenhansen
 */
public class TestSimplePhysicsCar {
    private static Vector3f wheelDirection=new Vector3f(0,-1,0);
    private static Vector3f wheelAxle=new Vector3f(-1,0,0);

    private static PhysicsVehicleNode physicsCar;

    public static void setupGame(){
        // creates and initializes the PhysicsSpace
        final PhysicsSpace pSpace=PhysicsSpace.getPhysicsSpace();

        // add some keybindings to control the vehicle
        KeyBindingManager.getKeyBindingManager().set("key_accelerate",
                KeyInput.KEY_U);
        KeyBindingManager.getKeyBindingManager().set("key_brake",
                KeyInput.KEY_J);
        KeyBindingManager.getKeyBindingManager().set("key_steer_left",
                KeyInput.KEY_H);
        KeyBindingManager.getKeyBindingManager().set("key_steer_right",
                KeyInput.KEY_K);

        // Create a DebugGameState
        // - override the update method to update/sync physics space
        DebugGameState state = new DebugGameState(){

            @Override
            public void update(float tpf) {
                pSpace.syncPhysics();
                pSpace.update(tpf);
                super.update(tpf);

                if (KeyBindingManager.getKeyBindingManager().isValidCommand(
                        "key_accelerate", false)) {
                    physicsCar.accelerate(1, true);
                }
                if (KeyBindingManager.getKeyBindingManager().isValidCommand(
                        "key_brake", false)) {
                    physicsCar.accelerate(0, false);
                    physicsCar.brake(.1f);
                }
                if (KeyBindingManager.getKeyBindingManager().isValidCommand(
                        "key_steer_left", true)) {
                    physicsCar.steer(.5f);
                }
                else if (KeyBindingManager.getKeyBindingManager().isValidCommand(
                        "key_steer_right", true)) {
                    physicsCar.steer(-.5f);
                }
                else{
                    physicsCar.steer(0);
                }

            }
            
        };

        // Add a physics vehicle to the world
        Box box1=new Box("physicscar",Vector3f.ZERO,0.5f,0.5f,2f);
        physicsCar=new PhysicsVehicleNode(box1,CollisionShape.Shapes.BOX);

        // Create four wheels and add them at their locations
        Sphere wheel=new Sphere("wheel",8,8,0.5f);
        physicsCar.addWheel(wheel, new Vector3f(-1f,-0.5f,2f), wheelDirection, wheelAxle, 0.2f, 0.5f, true);

        wheel=new Sphere("wheel",8,8,0.5f);
        physicsCar.addWheel(wheel, new Vector3f(1f,-0.5f,2f), wheelDirection, wheelAxle, 0.2f, 0.5f, true);

        wheel=new Sphere("wheel",8,8,0.5f);
        physicsCar.addWheel(wheel, new Vector3f(-1f,-0.5f,-2f), wheelDirection, wheelAxle, 0.2f, 0.5f, false);

        wheel=new Sphere("wheel",8,8,0.5f);
        physicsCar.addWheel(wheel, new Vector3f(1f,-0.5f,-2f), wheelDirection, wheelAxle, 0.2f, 0.5f, false);
        
//        physicsCar.setMass(100);
//        physicsCar.createCollisionShape(PhysicsNode.Shapes.BOX);
        physicsCar.setLocalTranslation(new Vector3f(10,-2,0));
        state.getRootNode().attachChild(physicsCar);
        physicsCar.updateRenderState();

        // an obstacle mesh, does not move (mass=0)
        PhysicsNode node2=new PhysicsNode(new Sphere("physicsobstaclemesh",16,16,1.2f),CollisionShape.Shapes.MESH);
        node2.setMass(0);
        node2.setLocalTranslation(new Vector3f(2.5f,-4,0f));
        state.getRootNode().attachChild(node2);
        node2.updateRenderState();

        // the floor, does not move (mass=0)
        PhysicsNode node3=new PhysicsNode(new Box("physicsfloor",Vector3f.ZERO,100f,0.2f,100f),CollisionShape.Shapes.MESH);
        node3.setMass(0);
        node3.setLocalTranslation(new Vector3f(0f,-6,0f));
        state.getRootNode().attachChild(node3);
        node3.updateRenderState();

        
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
