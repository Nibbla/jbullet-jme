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


import com.jme.input.InputHandler;
import com.jme.input.KeyBindingManager;
import com.jme.input.KeyInput;
import com.jme.input.action.InputAction;
import com.jme.input.action.InputActionEvent;
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
import com.jmex.jbullet.collision.shapes.CollisionShape;
import com.jmex.jbullet.nodes.PhysicsCharacterNode;
import com.jmex.jbullet.nodes.PhysicsNode;

/**
 * This is a basic Test of jbullet-jme character nodes
 *
 * @author normenhansen
 */
public class TestPhysicsCharacter {
    private static PhysicsCharacterNode character;
    private static Vector3f walkDirection=new Vector3f();

    public static void setupGame(){
        // creates and initializes the PhysicsSpace
        final PhysicsSpace pSpace=PhysicsSpace.getPhysicsSpace(PhysicsSpace.BroadphaseTypes.AXIS_SWEEP_3);

        // Create a DebugGameState
        // - override the update method to update/sync physics space
        DebugGameState state = new DebugGameState(){
            CollisionShape shape;

            private boolean isSetup=false;
            public void setupInputHandler(){
                if(isSetup) return;
                input.addAction( accelAction, InputHandler.DEVICE_KEYBOARD, KeyInput.KEY_U, InputHandler.AXIS_NONE, false);
                input.addAction( brakeAction, InputHandler.DEVICE_KEYBOARD, KeyInput.KEY_J, InputHandler.AXIS_NONE, false);
                input.addAction( steerLeftAction, InputHandler.DEVICE_KEYBOARD, KeyInput.KEY_H, InputHandler.AXIS_NONE, false);
                input.addAction( steerRightAction, InputHandler.DEVICE_KEYBOARD, KeyInput.KEY_K, InputHandler.AXIS_NONE, false);
                input.addAction( spaceAction, InputHandler.DEVICE_KEYBOARD, KeyInput.KEY_SPACE, InputHandler.AXIS_NONE, false);
                isSetup=true;
            }

            @Override
            public void update(float tpf) {
                //not very elegant: try to setup each update call
                setupInputHandler();
                
                super.update(tpf);
                pSpace.update(tpf);
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
    
    private static InputAction accelAction = new InputAction() {
        public void performAction( InputActionEvent evt ) {
            if(evt.getTriggerPressed()){
                walkDirection.addLocal(new Vector3f(0,0,-.1f));
            }
            else{
                walkDirection.addLocal(new Vector3f(0,0,.1f));
            }
            character.setWalkDirection(walkDirection);
        }
    };

    private static InputAction brakeAction = new InputAction() {
        public void performAction( InputActionEvent evt ) {
            if(evt.getTriggerPressed()){
                walkDirection.addLocal(new Vector3f(0,0,.1f));
            }
            else{
                walkDirection.addLocal(new Vector3f(0,0,-.1f));
            }
            character.setWalkDirection(walkDirection);
        }
    };

    private static InputAction steerLeftAction = new InputAction() {
        public void performAction( InputActionEvent evt ) {
            if(evt.getTriggerPressed()){
                walkDirection.addLocal(new Vector3f(-.1f,0,0));
            }
            else{
                walkDirection.addLocal(new Vector3f(.1f,0,0));
            }
            character.setWalkDirection(walkDirection);
        }
    };

    private static InputAction steerRightAction = new InputAction() {
        public void performAction( InputActionEvent evt ) {
            if(evt.getTriggerPressed()){
                walkDirection.addLocal(new Vector3f(.1f,0,0));
            }
            else{
                walkDirection.addLocal(new Vector3f(-.1f,0,0));
            }
            character.setWalkDirection(walkDirection);
        }
    };

    private static InputAction spaceAction = new InputAction() {
        public void performAction( InputActionEvent evt ) {
            if(evt.getTriggerPressed()){
                character.jump();
            }
            else{

            }
        }
    };


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
