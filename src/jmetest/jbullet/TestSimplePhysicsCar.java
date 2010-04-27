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
import com.jme.input.KeyInput;
import com.jme.input.action.InputAction;
import com.jme.input.action.InputActionEvent;
import com.jme.input.keyboard.KeyboardInputHandlerDevice;
import com.jmex.jbullet.collision.CollisionEvent;
import java.util.concurrent.Callable;

import com.jme.math.Vector3f;
import com.jme.renderer.Camera;
import com.jme.scene.shape.Box;
import com.jme.scene.shape.Sphere;
import com.jme.system.DisplaySystem;
import com.jme.util.GameTaskQueueManager;
import com.jmex.editors.swing.settings.GameSettingsPanel;
import com.jmex.game.StandardGame;
import com.jmex.game.state.DebugGameState;
import com.jmex.game.state.GameStateManager;
import com.jmex.jbullet.PhysicsSpace;
import com.jmex.jbullet.collision.CollisionListener;
import com.jmex.jbullet.collision.shapes.CollisionShape;
import com.jmex.jbullet.nodes.PhysicsVehicleNode;
import com.jmex.jbullet.nodes.PhysicsNode;

/**
 * This is a basic Test of jbullet-jme vehicles
 *
 * @author normenhansen
 */
public class TestSimplePhysicsCar{
    private static Vector3f wheelDirection=new Vector3f(0,-1,0);
    private static Vector3f wheelAxle=new Vector3f(-1,0,0);

    private static PhysicsVehicleNode physicsCar;
    private static DebugGameState state;
    private static PhysicsNode node2;
    private static boolean carCam=false;

    public static void setupGame(){
        // creates and initializes the PhysicsSpace
        final PhysicsSpace pSpace=PhysicsSpace.getPhysicsSpace();

        //collision listener for collisions with the sphere
        pSpace.addCollisionListener(new CollisionListener(){
            public void collision(CollisionEvent event) {
                if(event.getNodeA().equals(node2)&&event.getAppliedImpulse()>1){
                    state.setText("you hit the sphere! "+event.getAppliedImpulse());
                }
                else if(event.getNodeB().equals(node2)&&event.getAppliedImpulse()>1){
                    state.setText("you hit the sphere!!"+event.getAppliedImpulse());
                }
            }

        });

        // Create a DebugGameState
        // - override the update method to update/sync physics space
        state = new DebugGameState(){

            private boolean isSetup=false;
            public void setupInputHandler(){
                if(isSetup) return;
                input.addAction( accelAction, InputHandler.DEVICE_KEYBOARD, KeyInput.KEY_U, InputHandler.AXIS_NONE, false);
                input.addAction( brakeAction, InputHandler.DEVICE_KEYBOARD, KeyInput.KEY_J, InputHandler.AXIS_NONE, false);
                input.addAction( steerLeftAction, InputHandler.DEVICE_KEYBOARD, KeyInput.KEY_H, InputHandler.AXIS_NONE, false);
                input.addAction( steerRightAction, InputHandler.DEVICE_KEYBOARD, KeyInput.KEY_K, InputHandler.AXIS_NONE, false);
                input.addAction( spaceAction, InputHandler.DEVICE_KEYBOARD, KeyInput.KEY_SPACE, InputHandler.AXIS_NONE, false);
                input.addAction( escapeAction, InputHandler.DEVICE_KEYBOARD, KeyInput.KEY_ESCAPE, InputHandler.AXIS_NONE, false);
                input.addAction( viewAction, InputHandler.DEVICE_KEYBOARD, KeyInput.KEY_V, InputHandler.AXIS_NONE, false);
                isSetup=true;
            }
            
            @Override
            public void update(float tpf) {
                //not very elegant: try to setup each update call
                setupInputHandler();
                
                super.update(tpf);
                pSpace.update(tpf);
                if(carCam){
                    Camera cam=DisplaySystem.getDisplaySystem().getRenderer().getCamera();
                    cam.getLocation().set(physicsCar.getLocalTranslation().add(new Vector3f(0,1,0)));
                    cam.setAxes(physicsCar.getLocalRotation());
                }
            }

        };
        state.setText("u,h,j,k = control vehicle / v = toggle car camera / space = toggle upwards force to vehicle");

        // Add a physics vehicle to the world
        Box box1=new Box("physicscar",Vector3f.ZERO,0.5f,0.5f,2f);
        physicsCar=new PhysicsVehicleNode(box1,CollisionShape.ShapeTypes.BOX);
        physicsCar.setMaxSuspensionTravelCm(500);
        physicsCar.setSuspensionCompression(4.4f);
        physicsCar.setSuspensionDamping(2.3f);
        physicsCar.setSuspensionStiffness(20f);

        // Create four wheels and add them at their locations
        Sphere wheel=new Sphere("wheel",8,8,0.5f);
        physicsCar.addWheel(wheel, new Vector3f(-1f,-0.5f,2f), wheelDirection, wheelAxle, 0.2f, 0.5f, true);
        physicsCar.setRollInfluence(0, 1);

        wheel=new Sphere("wheel",8,8,0.5f);
        physicsCar.addWheel(wheel, new Vector3f(1f,-0.5f,2f), wheelDirection, wheelAxle, 0.2f, 0.5f, true);
        physicsCar.setRollInfluence(1, 1);

        wheel=new Sphere("wheel",8,8,0.5f);
        physicsCar.addWheel(wheel, new Vector3f(-1f,-0.5f,-2f), wheelDirection, wheelAxle, 0.2f, 0.5f, false);
        physicsCar.setRollInfluence(2, 1);

        wheel=new Sphere("wheel",8,8,0.5f);
        physicsCar.addWheel(wheel, new Vector3f(1f,-0.5f,-2f), wheelDirection, wheelAxle, 0.2f, 0.5f, false);
        physicsCar.setRollInfluence(3, 1);

        physicsCar.setLocalTranslation(new Vector3f(10,-2,0));
        state.getRootNode().attachChild(physicsCar);
        physicsCar.updateRenderState();
//        physicsCar.setMass(100);
        pSpace.add(physicsCar);
//        physicsCar.setMass(100);

        // an obstacle mesh, does not move (mass=0)
        node2=new PhysicsNode(new Sphere("physicsobstaclemesh",16,16,1.2f),CollisionShape.ShapeTypes.MESH,0);
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

    private static InputAction accelAction = new InputAction() {
        public void performAction( InputActionEvent evt ) {
            if(evt.getTriggerPressed()){
                physicsCar.accelerate(1);
            }
            else{
                physicsCar.accelerate(0);
            }
        }
    };

    private static InputAction brakeAction = new InputAction() {
        public void performAction( InputActionEvent evt ) {
            if(evt.getTriggerPressed()){
                physicsCar.brake(.1f);
            }
            else{
                physicsCar.brake(0);
            }
        }
    };

    private static boolean leftLast=false;
    private static InputAction steerLeftAction = new InputAction() {
        public void performAction( InputActionEvent evt ) {
            if(evt.getTriggerPressed()){
                leftLast=true;
                physicsCar.steer(.5f);
            }
            else{
                if(leftLast)
                    physicsCar.steer(0);
            }
        }
    };

    private static InputAction steerRightAction = new InputAction() {
        public void performAction( InputActionEvent evt ) {
            if(evt.getTriggerPressed()){
                leftLast=false;
                physicsCar.steer(-.5f);
            }
            else{
                if(!leftLast)
                    physicsCar.steer(0);
            }
        }
    };

    private static InputAction spaceAction = new InputAction() {
        public void performAction( InputActionEvent evt ) {
            if(evt.getTriggerPressed()){
                if(physicsCar.getContinuousForce()==null)
                    physicsCar.applyContinuousForce(true, Vector3f.UNIT_Y.mult(10));
                else
                    physicsCar.applyContinuousForce(false);
            }
            else{

            }
        }
    };

    private static InputAction escapeAction = new InputAction() {
        public void performAction( InputActionEvent evt ) {
            if(evt.getTriggerPressed()){

            }
            else{

            }
        }
    };

    private static InputAction viewAction = new InputAction() {
        public void performAction( InputActionEvent evt ) {
            if(evt.getTriggerPressed()){
                carCam=!carCam;
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
