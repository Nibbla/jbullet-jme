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


import com.jme.math.FastMath;
import com.jme.math.Quaternion;
import java.util.concurrent.Callable;

import com.jme.math.Vector3f;
import com.jme.scene.shape.Box;
import com.jme.scene.shape.Capsule;
import com.jme.util.GameTaskQueueManager;
import com.jmex.editors.swing.settings.GameSettingsPanel;
import com.jmex.game.StandardGame;
import com.jmex.game.state.DebugGameState;
import com.jmex.game.state.GameStateManager;
import com.jmex.jbullet.PhysicsSpace;
import com.jmex.jbullet.collision.CollisionShape;
import com.jmex.jbullet.joints.PhysicsConeJoint;
import com.jmex.jbullet.nodes.PhysicsNode;

/**
 * This is a basic jbullet-jme RagDoll test (TODO)
 *
 * @author normenhansen
 */
public class TestSimpleRagdoll {

    public static void setupGame(){
        // creates and initializes the PhysicsSpace
        final PhysicsSpace pSpace=PhysicsSpace.getPhysicsSpace();

        // Create a DebugGameState
        // - override the update method to update/sync physics space
        DebugGameState state = new DebugGameState(){

            @Override
            public void update(float tpf) {
                pSpace.syncPhysics();
                pSpace.update(tpf);
                super.update(tpf);
            }
            
        };


        //create limb nodes
        Capsule shoulders=new Capsule("capsule",8,8,8,0.2f,2f);
        PhysicsNode shouldersNode=new PhysicsNode(shoulders,CollisionShape.Shapes.CAPSULE);
        shouldersNode.setLocalTranslation(0, 0, 0);
        shouldersNode.setLocalRotation(new Quaternion().fromAngleAxis(FastMath.PI / 2, Vector3f.UNIT_Z));

        Capsule lArmLower=new Capsule("capsule",8,8,8,0.2f,1f);
        PhysicsNode lArmLowerNode=new PhysicsNode(lArmLower,CollisionShape.Shapes.CAPSULE);
        lArmLowerNode.setLocalTranslation(-1, -2, 0);
        Capsule lArmUpper=new Capsule("capsule",8,8,8,0.2f,1f);
        PhysicsNode lArmUpperNode=new PhysicsNode(lArmUpper,CollisionShape.Shapes.CAPSULE);
        lArmUpperNode.setLocalTranslation(-1, -1, 0);
        Capsule rArmLower=new Capsule("capsule",8,8,8,0.2f,1f);
        PhysicsNode rArmLowerNode=new PhysicsNode(rArmLower,CollisionShape.Shapes.CAPSULE);
        rArmLowerNode.setLocalTranslation(1, -2, 0);
        Capsule rArmUpper=new Capsule("capsule",8,8,8,0.2f,1f);
        PhysicsNode rArmUpperNode=new PhysicsNode(rArmUpper,CollisionShape.Shapes.CAPSULE);
        rArmUpperNode.setLocalTranslation(1, -1, 0);

        Capsule body=new Capsule("capsule",8,8,8,0.2f,2f);
        PhysicsNode bodyNode=new PhysicsNode(body,CollisionShape.Shapes.CAPSULE);
        bodyNode.setLocalTranslation(0, -1, 0);

        Capsule hips=new Capsule("capsule",8,8,8,0.2f,1f);
        PhysicsNode hipsNode=new PhysicsNode(hips,CollisionShape.Shapes.CAPSULE);
        hipsNode.setLocalTranslation(0, -2, 0);
        hipsNode.setLocalRotation(new Quaternion().fromAngleAxis(FastMath.PI / 2, Vector3f.UNIT_Z));

        Capsule lLegLower=new Capsule("capsule",8,8,8,0.2f,1f);
        PhysicsNode lLegLowerNode=new PhysicsNode(lLegLower,CollisionShape.Shapes.CAPSULE);
        lLegLowerNode.setLocalTranslation(-0.5f, -4, 0);
        Capsule lLegUpper=new Capsule("capsule",8,8,8,0.2f,1f);
        PhysicsNode lLegUpperNode=new PhysicsNode(lLegUpper,CollisionShape.Shapes.CAPSULE);
        lLegUpperNode.setLocalTranslation(-0.5f, -3, 0);
        Capsule rLegLower=new Capsule("capsule",8,8,8,0.2f,1f);
        PhysicsNode rLegLowerNode=new PhysicsNode(rLegLower,CollisionShape.Shapes.CAPSULE);
        rLegLowerNode.setLocalTranslation(0.5f, -4, 0);
        Capsule rLegUpper=new Capsule("capsule",8,8,8,0.2f,1f);
        PhysicsNode rLegUpperNode=new PhysicsNode(rLegUpper,CollisionShape.Shapes.CAPSULE);
        rLegUpperNode.setLocalTranslation(0.5f, -3, 0);

        //add nodes to scenegraph & physics space
        state.getRootNode().attachChild(shouldersNode);
        shouldersNode.updateRenderState();
        pSpace.add(shouldersNode);
        state.getRootNode().attachChild(lArmLowerNode);
        lArmLowerNode.updateRenderState();
        pSpace.add(lArmLowerNode);
        state.getRootNode().attachChild(lArmUpperNode);
        lArmUpperNode.updateRenderState();
        pSpace.add(lArmUpperNode);
        state.getRootNode().attachChild(rArmLowerNode);
        rArmLowerNode.updateRenderState();
        pSpace.add(rArmLowerNode);
        state.getRootNode().attachChild(rArmUpperNode);
        rArmUpperNode.updateRenderState();
        pSpace.add(rArmUpperNode);

        state.getRootNode().attachChild(bodyNode);
        bodyNode.updateRenderState();
        pSpace.add(bodyNode);

        state.getRootNode().attachChild(hipsNode);
        hipsNode.updateRenderState();
        pSpace.add(hipsNode);

        state.getRootNode().attachChild(lLegLowerNode);
        lLegLowerNode.updateRenderState();
        pSpace.add(lLegLowerNode);
        state.getRootNode().attachChild(lLegUpperNode);
        lLegUpperNode.updateRenderState();
        pSpace.add(lLegUpperNode);
        state.getRootNode().attachChild(rLegLowerNode);
        rLegLowerNode.updateRenderState();
        pSpace.add(rLegLowerNode);
        state.getRootNode().attachChild(rLegUpperNode);
        rLegUpperNode.updateRenderState();
        pSpace.add(rLegUpperNode);

        //create/add joints. TODO: limits
        PhysicsConeJoint lShoulderJoint=new PhysicsConeJoint(shouldersNode, lArmUpperNode, new Vector3f(0,-1,0), new Vector3f(0,0.5f,0));
        lShoulderJoint.setCollisionBetweenLinkedBodys(false);
        lShoulderJoint.setLimit(0f, 0f, 0f);
        pSpace.add(lShoulderJoint);
        PhysicsConeJoint lArmJoint=new PhysicsConeJoint(lArmUpperNode, lArmLowerNode, new Vector3f(0,-0.5f,0), new Vector3f(0,0.5f,0));
        lArmJoint.setCollisionBetweenLinkedBodys(false);
        lArmJoint.setLimit(0f, 0f, 0f);
        pSpace.add(lArmJoint);

        PhysicsConeJoint rShoulderJoint=new PhysicsConeJoint(shouldersNode, rArmUpperNode, new Vector3f(0,1,0), new Vector3f(0,0.5f,0));
        rShoulderJoint.setCollisionBetweenLinkedBodys(false);
        rShoulderJoint.setLimit(0f, 0f, 0f);
        pSpace.add(rShoulderJoint);
        PhysicsConeJoint rArmJoint=new PhysicsConeJoint(rArmUpperNode, rArmLowerNode, new Vector3f(0,-0.5f,0), new Vector3f(0,0.5f,0));
        rArmJoint.setCollisionBetweenLinkedBodys(false);
        rArmJoint.setLimit(0f, 0f, 0f);
        pSpace.add(rArmJoint);

        PhysicsConeJoint bodyJoint=new PhysicsConeJoint(shouldersNode, bodyNode, new Vector3f(0,0,0), new Vector3f(0,1f,0));
        bodyJoint.setCollisionBetweenLinkedBodys(false);
        bodyJoint.setLimit(0f, 0f, 0f);
        pSpace.add(bodyJoint);

        PhysicsConeJoint hipJoint=new PhysicsConeJoint(bodyNode, hipsNode, new Vector3f(0,-1,0), new Vector3f(0,0,0));
        hipJoint.setCollisionBetweenLinkedBodys(false);
        hipJoint.setLimit(0f, 0f, 0f);
        pSpace.add(hipJoint);

        PhysicsConeJoint lLeg=new PhysicsConeJoint(hipsNode, lLegUpperNode, new Vector3f(0,-0.5f,0), new Vector3f(0,0.5f,0));
        lLeg.setCollisionBetweenLinkedBodys(false);
        lLeg.setLimit(0f, 0f, 0f);
        pSpace.add(lLeg);
        PhysicsConeJoint lLegLow=new PhysicsConeJoint(lLegUpperNode, lLegLowerNode, new Vector3f(0,-0.5f,0), new Vector3f(0,0.5f,0));
        lLegLow.setCollisionBetweenLinkedBodys(false);
        lLegLow.setLimit(0f, 0f, 0f);
        pSpace.add(lLegLow);

        PhysicsConeJoint rLeg=new PhysicsConeJoint(hipsNode, rLegUpperNode, new Vector3f(0,0.5f,0), new Vector3f(0,0.5f,0));
        rLeg.setCollisionBetweenLinkedBodys(false);
        rLeg.setLimit(0f, 0f, 0f);
        pSpace.add(rLeg);
        PhysicsConeJoint rLegLow=new PhysicsConeJoint(rLegUpperNode, rLegLowerNode, new Vector3f(0,-0.5f,0), new Vector3f(0,0.5f,0));
        rLegLow.setCollisionBetweenLinkedBodys(false);
        rLegLow.setLimit(0f, 0f, 0f);
        pSpace.add(rLegLow);

        // the floor, does not move (mass=0)
        PhysicsNode physicsFloor=new PhysicsNode(new Box("physicsfloor",Vector3f.ZERO,100f,0.2f,100f),CollisionShape.Shapes.MESH);
        physicsFloor.setMass(0);
        physicsFloor.setLocalTranslation(new Vector3f(0f,-6,0f));
        state.getRootNode().attachChild(physicsFloor);
        physicsFloor.updateRenderState();
        pSpace.add(physicsFloor);

        
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
