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
package com.jmex.jbullet;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.ContactAddedCallback;
import com.bulletphysics.ContactDestroyedCallback;
import com.bulletphysics.ContactProcessedCallback;
import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.jme.math.Vector3f;
import com.jme.util.GameTaskQueue;
import com.jmex.jbullet.joints.PhysicsJoint;
import com.jmex.jbullet.node.PhysicsVehicleNode;
import com.jmex.jbullet.node.PhysicsNode;
import java.util.LinkedList;
import java.util.List;

/**
 * <p>
 * <b>About jbullet-jme</b><br>
 * <i>
 * jbullet-jme is a wrapper of jBullet for jme2. It aims at simplifying the use
 * of the jbullet/bullet physics library in jme2 projects.<br>
 * It does so by integrating jbullet features into a set of simple to use phyiscs objects
 * that play nicely with jme2.<br>
 * Other than jmephysics2, which aims at integrating different physics engines
 * in one wrapper library, jbullet-jme is more or less tailored to bullet.
 * This allows using some of the special functions of bullet (like vehicles)
 * but makes the wrapper more difficult to port to other physics implementations.
 * Still, wrapping jbullet like its done here keeps the possibility of developing a
 * native bullet implementation of jbullet-jme in the future, supporting SPU use etc.
 * <br>Project Status: <b>pre-alpha</b>
 * </i>
 * </p>
 * <p>PhysicsSpace - The central jbullet-jme physics space</p>
 * <p>
 * USAGE:<br>
 * The PhysicsSpace needs to be started by creating a <code>PhysicsSpace</code> Object
 * and updated by calling the update(float time) method. The syncPhysics() method
 * synchronizes the physics and jme objects and is called seperately to allow multithreading
 * and/or different decoupling techniques between the OpenGL and Physics engine.<br>
 * Bullet internally works with 60 frames/s...
 * TODO: extend
 *
 * </p>
 * <p>
 * Currently Implemented: (but all TODO)<br>
 * - RigidBodys (via PhysicsNode)<br>
 * - Applying forces to PhysicsNodes<br>
 * - Vehicles using bullet RaycastVehicle (via PhysicsVehicleNode)<br>
 * - Joints: Point2Point, Hinge<br>
 * </p>
 * <p>
 * ISSUES:<br>
 * - Currently, mesh objects DO NOT collide with other mesh objects<br>
 * </p>
 * <p>
 * TODO:<br>
 * - Callbacks<br>
 * - Characters using bullet KinematicCharacterController (via PhysicsCharacterNode)<br>
 * - Joints: Cone, Slider, 6Dof<br>
 * - Performance settings (world size, used Solver/Dispatcher etc.)<br>
 * - JGN integration<br>
 * - Dynamic updates: Updates each physics tick, not world tick.. how to wrap?<br>
 * - check JNI/native compatibility of API (pointers to pSpace/rBody in java, these will be native..)<br>
 * </p>
 * @see com.jmex.jbullet.node.PhysicsNode
 * @author normenhansen
 */
public class PhysicsSpace {
    private GameTaskQueue pQueue;
    private DynamicsWorld dynamicsWorld = null;
    private BroadphaseInterface broadphase;
    private CollisionDispatcher dispatcher;
    private ConstraintSolver solver;
    private DefaultCollisionConfiguration collisionConfiguration;

    private List<PhysicsNode> physicsNodes=new LinkedList<PhysicsNode>();
    private List<PhysicsJoint> physicsJoints=new LinkedList<PhysicsJoint>();

    public PhysicsSpace(){
        //TODO: better multithreading/updating support via queues
//        GameTaskQueueManager.getManager().addQueue("jbullet_update", new GameTaskQueue());
//        pQueue=GameTaskQueueManager.getManager().getQueue("jbullet_update");
//        pQueue.setExecuteAll(true);

        collisionConfiguration = new DefaultCollisionConfiguration();
        dispatcher = new CollisionDispatcher( collisionConfiguration );
//        broadphase = new SimpleBroadphase();
        broadphase = new DbvtBroadphase();
        solver = new SequentialImpulseConstraintSolver();

        dynamicsWorld = new DiscreteDynamicsWorld( dispatcher, broadphase, solver, collisionConfiguration );
        dynamicsWorld.setGravity( new javax.vecmath.Vector3f( 0, -9.81f, 0 ) );
        
        setContactCallbacks();

    }

    public void update(float time){
        update(time,1);
    }

    public void update(float time, int maxSteps){
        if(getDynamicsWorld()==null) return;
        getDynamicsWorld().stepSimulation(time,maxSteps);
        
    }

    public void syncPhysics(){
        for ( PhysicsNode node : physicsNodes ){
            node.syncPhysics();
        }
        for (PhysicsJoint joint : physicsJoints){
            joint.syncPhysics();
        }
    }

    /**
     * used from the constructor of physics objects, no need to call manually
     */
    public void addNode(PhysicsNode node){
        physicsNodes.add(node);
        getDynamicsWorld().addRigidBody(node.getRigidBody());
        if(node instanceof PhysicsVehicleNode)
            dynamicsWorld.addVehicle(((PhysicsVehicleNode)node).getVehicle());
    }

    /**
     * used from the destroy() method of physics objects, no need to call manually
     */
    public void removeNode(PhysicsNode node){
        physicsNodes.remove(node);
        getDynamicsWorld().removeRigidBody(node.getRigidBody());
    }

    public void addJoint(PhysicsJoint joint){
        physicsJoints.add(joint);
        getDynamicsWorld().addConstraint(joint.getConstraint());
    }

    public void setGravity(Vector3f gravity){
        throw (new UnsupportedOperationException("Not implemented yet."));
    }

    private void setContactCallbacks() {
        BulletGlobals.setContactAddedCallback(new ContactAddedCallback(){
        	public boolean contactAdded(ManifoldPoint cp, CollisionObject colObj0,
        			int partId0, int index0, CollisionObject colObj1, int partId1,
        			int index1){
                System.out.println("contact added");
        		return true;
        	}
        });

        BulletGlobals.setContactProcessedCallback(new ContactProcessedCallback(){
        	public boolean contactProcessed(ManifoldPoint cp, Object body0, Object body1){
        		if(cp.userPersistentData==null){
//                    System.out.println("contact processed, no perdata: "+body0+"/"+body1);
        			return true;
                }
                System.out.println("contact processed");
        		return true;
        	}
    	});

        BulletGlobals.setContactDestroyedCallback(new ContactDestroyedCallback(){
 			public boolean contactDestroyed(Object userPersistentData) {
                System.out.println("contact destroyed");
 				return true;
			}
    	});
    }

    /**
     * @return the dynamicsWorld
     */
    public DynamicsWorld getDynamicsWorld() {
        return dynamicsWorld;
    }

}