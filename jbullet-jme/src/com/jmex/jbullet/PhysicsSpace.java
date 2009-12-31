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
package com.jmex.jbullet;

import com.bulletphysics.BulletGlobals;
import com.bulletphysics.ContactAddedCallback;
import com.bulletphysics.ContactDestroyedCallback;
import com.bulletphysics.ContactProcessedCallback;
import com.bulletphysics.collision.broadphase.BroadphaseInterface;
import com.bulletphysics.collision.broadphase.DbvtBroadphase;
import com.bulletphysics.collision.dispatch.CollisionDispatcher;
import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
import com.bulletphysics.collision.dispatch.GhostObject;
import com.bulletphysics.collision.narrowphase.ManifoldPoint;
import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.dynamics.RigidBody;
import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.jme.math.Vector3f;
import com.jme.util.GameTaskQueue;
import com.jme.util.GameTaskQueueManager;
import com.jmex.jbullet.collision.CollisionEvent;
import com.jmex.jbullet.collision.CollisionListener;
import com.jmex.jbullet.collision.CollisionObject;
import com.jmex.jbullet.joints.PhysicsJoint;
import com.jmex.jbullet.nodes.PhysicsGhostNode;
import com.jmex.jbullet.nodes.PhysicsVehicleNode;
import com.jmex.jbullet.nodes.PhysicsNode;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.concurrent.ConcurrentHashMap;

/**
 * <p>PhysicsSpace - The central jbullet-jme physics space</p>
 * <p>
 * USAGE:<br>
 * The PhysicsSpace needs to be started by getting a <code>PhysicsSpace</code> Object
 * by calling <code>PhysicsSpace physics=PhysicsSpace.getPhysicsSpace();</code>
 * and updated by calling the update(float time) method. The syncPhysics() method
 * synchronizes the physics and jme objects and is called seperately to allow multithreading
 * and/or different decoupling techniques between the OpenGL and Physics engine.<br>
 * Bullet internally works with 60 frames/s...<br>
 * TODO: extend
 * </p>
 * @see com.jmex.jbullet.node.PhysicsNode
 * @author normenhansen
 */
public class PhysicsSpace {
    private GameTaskQueue rQueue;
    private GameTaskQueue pQueue;
    private DynamicsWorld dynamicsWorld = null;
    private BroadphaseInterface broadphase;
    private CollisionDispatcher dispatcher;
    private ConstraintSolver solver;
    private DefaultCollisionConfiguration collisionConfiguration;

    private Map<GhostObject,PhysicsGhostNode> physicsGhostNodes=new ConcurrentHashMap<GhostObject,PhysicsGhostNode>();
    private Map<RigidBody,PhysicsNode> physicsNodes=new ConcurrentHashMap<RigidBody,PhysicsNode>();
    private List<PhysicsJoint> physicsJoints=new LinkedList<PhysicsJoint>();

    private List<CollisionListener> collisionListeners=new LinkedList<CollisionListener>();
    private List<CollisionEvent> collisionEvents=new LinkedList<CollisionEvent>();

    private static PhysicsSpace pSpace;
    
    public static PhysicsSpace getPhysicsSpace(){
        if(pSpace!=null){
            return pSpace;
        }
        pSpace=new PhysicsSpace();
        return pSpace;
    }

    public void enqueueSync(Callable callable){
        pQueue.enqueue(callable);
    }

    public void reQueue(final Callable callable){
        rQueue.enqueue(new Callable(){
            public Object call() throws Exception {
                pQueue.enqueue(callable);
                return null;
            }

        });
    }

    public PhysicsSpace(){
        //TODO: better multithreading/updating support via queues
        GameTaskQueueManager.getManager().addQueue("jbullet_requeue", new GameTaskQueue());
        rQueue=GameTaskQueueManager.getManager().getQueue("jbullet_requeue");
        rQueue.setExecuteAll(true);

        GameTaskQueueManager.getManager().addQueue("jbullet_sync", new GameTaskQueue());
        pQueue=GameTaskQueueManager.getManager().getQueue("jbullet_sync");
        pQueue.setExecuteAll(true);

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
        //distribute events
        distributeEvents();
        //add recurring events
        rQueue.execute();
        //sync nodes+joints
        pQueue.execute();
        //sync ghostnodes
        for ( PhysicsGhostNode node : physicsGhostNodes.values() ){
            node.syncPhysics();
        }
    }

    private void distributeEvents() {
        //add collision callbacks
        for(CollisionListener listener:collisionListeners){
            for(CollisionEvent event:collisionEvents)
                listener.collision(event);
        }
        collisionEvents.clear();
    }

    /**
     * adds an object to the physics space
     * @param obj the PhyiscsNode, PhysicsGhostNode or PhysicsJoint to add
     */
    public void add(Object obj){
        if(obj instanceof PhysicsGhostNode){
            addGhostNode((PhysicsGhostNode)obj);
        }
        else if(obj instanceof PhysicsNode){
            addNode((PhysicsNode)obj);
        }
        else if(obj instanceof PhysicsJoint){
            addJoint((PhysicsJoint)obj);
        }
        else{
            throw (new UnsupportedOperationException("Cannot add this kind of object to the physics space."));
        }
    }

    /**
     * adds an object to the physics space
     * @param obj the PhyiscsNode, PhysicsGhostNode or PhysicsJoint to remove
     */
    public void remove(Object obj){
        if(obj instanceof PhysicsGhostNode){
            removeGhostNode((PhysicsGhostNode)obj);
        }
        else if(obj instanceof PhysicsNode){
            removeNode((PhysicsNode)obj);
        }
        else if(obj instanceof PhysicsJoint){
            removeJoint((PhysicsJoint)obj);
        }
        else{
            throw (new UnsupportedOperationException("Cannot remove this kind of object to the physics space."));
        }
    }

    private void addGhostNode(PhysicsGhostNode node){
        node.syncPhysics();
        physicsGhostNodes.put(node.getGhostObject(),node);
        getDynamicsWorld().addCollisionObject(node.getGhostObject());
    }

    private void removeGhostNode(PhysicsGhostNode node){
        physicsGhostNodes.remove(node.getGhostObject());
        getDynamicsWorld().removeCollisionObject(node.getGhostObject());
    }

    private void addNode(PhysicsNode node){
        node.syncPhysics();
        physicsNodes.put(node.getRigidBody(),node);
        getDynamicsWorld().addRigidBody(node.getRigidBody());
        if(node instanceof PhysicsVehicleNode)
            dynamicsWorld.addVehicle(((PhysicsVehicleNode)node).getVehicle());
    }

    private void removeNode(PhysicsNode node){
        physicsNodes.remove(node.getRigidBody());
        getDynamicsWorld().removeRigidBody(node.getRigidBody());
        if(node instanceof PhysicsVehicleNode)
            dynamicsWorld.removeVehicle(((PhysicsVehicleNode)node).getVehicle());
    }

    private void addJoint(PhysicsJoint joint){
        joint.syncPhysics();
        physicsJoints.add(joint);
        getDynamicsWorld().addConstraint(joint.getConstraint(), !joint.isCollisionBetweenLinkedBodys());
    }

    private void removeJoint(PhysicsJoint joint){
        physicsJoints.remove(joint);
        getDynamicsWorld().removeConstraint(joint.getConstraint());
    }

    /**
     * not safe
     * @param gravity
     */
    public void setGravity(Vector3f gravity){
        pSpace.setGravity(gravity);
    }

    public void addCollisionListener(CollisionListener listener){
        collisionListeners.add(listener);
    }

    public void removeCollisionListener(CollisionListener listener){
        collisionListeners.remove(listener);
    }

    private void setContactCallbacks() {
        BulletGlobals.setContactAddedCallback(new ContactAddedCallback(){
        	public boolean contactAdded(ManifoldPoint cp, com.bulletphysics.collision.dispatch.CollisionObject colObj0,
        			int partId0, int index0, com.bulletphysics.collision.dispatch.CollisionObject colObj1, int partId1,
        			int index1){
                System.out.println("contact added");
        		return true;
        	}
        });

        BulletGlobals.setContactProcessedCallback(new ContactProcessedCallback(){
        	public boolean contactProcessed(ManifoldPoint cp, Object body0, Object body1){
                CollisionObject node=null,node1=null;
                if(body0 instanceof RigidBody){
                    RigidBody rBody=(RigidBody)body0;
                    node=physicsNodes.get(rBody);
                }
                else if(body0 instanceof GhostObject){
                    GhostObject rBody=(GhostObject)body0;
                    node=physicsGhostNodes.get(rBody);
                }
                if(body1 instanceof RigidBody){
                    RigidBody rBody=(RigidBody)body1;
                    node1=physicsNodes.get(rBody);
                }
                else if(body1 instanceof GhostObject){
                    GhostObject rBody=(GhostObject)body1;
                    node1=physicsGhostNodes.get(rBody);
                }
                if(node!=null&&node1!=null)
                    collisionEvents.add(new CollisionEvent(CollisionEvent.TYPE_PROCESSED,node,node1,cp));
                else
                    System.out.println("error finding node during collision");
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