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
package com.jmex.bullet;

//import com.bulletphysics.BulletGlobals;
//import com.bulletphysics.ContactAddedCallback;
//import com.bulletphysics.ContactDestroyedCallback;
//import com.bulletphysics.ContactProcessedCallback;
//import com.bulletphysics.collision.broadphase.AxisSweep3;
//import com.bulletphysics.collision.broadphase.AxisSweep3_32;
//import com.bulletphysics.collision.broadphase.BroadphaseInterface;
//import com.bulletphysics.collision.broadphase.DbvtBroadphase;
//import com.bulletphysics.collision.broadphase.SimpleBroadphase;
//import com.bulletphysics.collision.dispatch.CollisionDispatcher;
//import com.bulletphysics.collision.dispatch.DefaultCollisionConfiguration;
//import com.bulletphysics.collision.dispatch.GhostObject;
//import com.bulletphysics.collision.dispatch.GhostPairCallback;
//import com.bulletphysics.collision.narrowphase.ManifoldPoint;
//import com.bulletphysics.dynamics.DiscreteDynamicsWorld;
//import com.bulletphysics.dynamics.DynamicsWorld;
//import com.bulletphysics.dynamics.RigidBody;
//import com.bulletphysics.dynamics.constraintsolver.ConstraintSolver;
//import com.bulletphysics.dynamics.constraintsolver.SequentialImpulseConstraintSolver;
import com.jme.math.Vector3f;
import com.jme.util.GameTaskQueue;
import com.jme.util.GameTaskQueueManager;
import com.jmex.bullet.collision.CollisionEvent;
import com.jmex.bullet.collision.CollisionListener;
import com.jmex.bullet.collision.CollisionObject;
import com.jmex.bullet.joints.PhysicsJoint;
import com.jmex.bullet.nodes.PhysicsGhostNode;
import com.jmex.bullet.nodes.PhysicsCharacterNode;
import com.jmex.bullet.nodes.PhysicsVehicleNode;
import com.jmex.bullet.nodes.PhysicsNode;
//import com.jmex.jbullet.util.Converter;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.Callable;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.Future;

/**
 * <p>PhysicsSpace - The central jbullet-jme physics space</p>
 * <p>
 * USAGE:<br>
 * The PhysicsSpace needs to be started by getting a <code>PhysicsSpace</code> Object
 * by calling <code>PhysicsSpace physics=PhysicsSpace.getPhysicsSpace();</code>
 * and updated by calling the <code>update(float time)</code> method.<br>
 * Bullet internally works with 60 frames/s...<br>
 * TODO: extend
 * </p>
 * @see com.jmex.jbullet.nodes.PhysicsNode
 * @author normenhansen
 */
public abstract class PhysicsSpace {
    static{
        System.loadLibrary("bullet-jme");
    }
    private GameTaskQueue rQueue;
    private GameTaskQueue pQueue;
//    private DynamicsWorld dynamicsWorld = null;
//    private BroadphaseInterface broadphase;
    private int broadphaseType=0;
//    private CollisionDispatcher dispatcher;
//    private ConstraintSolver solver;
//    private DefaultCollisionConfiguration collisionConfiguration;
//
    private Map<Long,PhysicsGhostNode> physicsGhostNodes=new ConcurrentHashMap<Long,PhysicsGhostNode>();
    private Map<Long,PhysicsNode> physicsNodes=new ConcurrentHashMap<Long,PhysicsNode>();
    private List<PhysicsJoint> physicsJoints=new LinkedList<PhysicsJoint>();

    private List<CollisionListener> collisionListeners=new LinkedList<CollisionListener>();
    private List<CollisionEvent> collisionEvents=new LinkedList<CollisionEvent>();

    private static PhysicsSpace pSpace;

    private Vector3f worldMin = new Vector3f(-10000f,-10000f,-10000f);
    private Vector3f worldMax = new Vector3f(10000f,10000f,10000f);

    /**
     * Get the current PhysicsSpace or create a new standard PhysicsSpace
     * @return the exising or created PhysicsSpace
     */
    public static PhysicsSpace getPhysicsSpace(){
        if(pSpace!=null){
            return pSpace;
        }
        pSpace=new PhysicsSpace(){};
        return pSpace;
    }

    /**
     * Get the current PhysicsSpace or create a new PhysicsSpace with
     * the given Broadphase type.
     * @param broadphaseType The PhysicsSpace.BroadphaseTypes.TYPE of the Boradphase to use
     * @return the exising or created PhysicsSpace
     */
    public static PhysicsSpace getPhysicsSpace(int broadphaseType){
        if(pSpace!=null){
            return pSpace;
        }
        pSpace=new PhysicsSpace(broadphaseType){};
        return pSpace;
    }

    /**
     * Get the current PhysicsSpace or create a new PhysicsSpace with
     * the AxisSweep3 Broadphase type and given world size.
     * @param worldMin the worldMin vector (e.g. -1000,-1000,-1000)
     * @param worldMax the worldMax vector (e.g. -1000,-1000,-1000)
     * @return the exising or created PhysicsSpace
     */
    public static PhysicsSpace getPhysicsSpace(Vector3f worldMin, Vector3f worldMax){
        if(pSpace!=null){
            return pSpace;
        }
        pSpace=new PhysicsSpace(worldMin, worldMax){};
        return pSpace;
    }

    /**
     * Get the current PhysicsSpace or create a new PhysicsSpace with
     * the given Broadphase type and given world size.
     * @param worldMin the worldMin vector (e.g. -1000,-1000,-1000)
     * @param worldMax the worldMax vector (e.g. -1000,-1000,-1000)
     * @param broadphaseType The PhysicsSpace.BroadphaseTypes.TYPE of the Boradphase to use
     * @return the exising or created PhysicsSpace
     */
    public static PhysicsSpace getPhysicsSpace(Vector3f worldMin, Vector3f worldMax, int broadphaseType){
        if(pSpace!=null){
            return pSpace;
        }
        pSpace=new PhysicsSpace(worldMin, worldMax, broadphaseType){};
        return pSpace;
    }

    public PhysicsSpace(){
        this(new Vector3f(-10000f,-10000f,-10000f),new Vector3f(10000f,10000f,10000f),BroadphaseTypes.SIMPLE);
    }

    public PhysicsSpace(int broadphaseType){
        this(new Vector3f(-10000f,-10000f,-10000f),new Vector3f(10000f,10000f,10000f),broadphaseType);
    }

    public PhysicsSpace(Vector3f worldMin, Vector3f worldMax){
        this(worldMin,worldMax,BroadphaseTypes.AXIS_SWEEP_3);
    }

    public PhysicsSpace(Vector3f worldMin, Vector3f worldMax, int broadphaseType){
        this.worldMin.set(worldMin);
        this.worldMax.set(worldMax);
        this.broadphaseType=broadphaseType;
        GameTaskQueueManager.getManager().addQueue("jbullet_requeue", new GameTaskQueue());
        rQueue=GameTaskQueueManager.getManager().getQueue("jbullet_requeue");
        rQueue.setExecuteAll(true);

        GameTaskQueueManager.getManager().addQueue("jbullet_update", new GameTaskQueue());
        pQueue=GameTaskQueueManager.getManager().getQueue("jbullet_update");
        pQueue.setExecuteAll(true);
//		broadphase.getOverlappingPairCache().setInternalGhostPairCallback(new GhostPairCallback());
        startPhysicsSpace(worldMin.x,worldMin.y,worldMin.z,worldMax.x,worldMax.y,worldMax.z,broadphaseType);
        setContactCallbacks();
    }

    native boolean startPhysicsSpace(float minX, float minY, float minZ, float maxX, float maxY, float maxZ, int broadphaseType);

    private void setContactCallbacks() {
//        BulletGlobals.setContactAddedCallback(new ContactAddedCallback(){
//        	public boolean contactAdded(ManifoldPoint cp, com.bulletphysics.collision.dispatch.CollisionObject colObj0,
//        			int partId0, int index0, com.bulletphysics.collision.dispatch.CollisionObject colObj1, int partId1,
//        			int index1){
//                System.out.println("contact added");
//        		return true;
//        	}
//        });
//
//        BulletGlobals.setContactProcessedCallback(new ContactProcessedCallback(){
//        	public boolean contactProcessed(ManifoldPoint cp, Object body0, Object body1){
//                CollisionObject node=null,node1=null;
//                if(body0 instanceof RigidBody){
//                    RigidBody rBody=(RigidBody)body0;
//                    node=physicsNodes.get(rBody);
//                }
//                else if(body0 instanceof GhostObject){
//                    GhostObject rBody=(GhostObject)body0;
//                    node=physicsGhostNodes.get(rBody);
//                }
//                if(body1 instanceof RigidBody){
//                    RigidBody rBody=(RigidBody)body1;
//                    node1=physicsNodes.get(rBody);
//                }
//                else if(body1 instanceof GhostObject){
//                    GhostObject rBody=(GhostObject)body1;
//                    node1=physicsGhostNodes.get(rBody);
//                }
//                if(node!=null&&node1!=null)
//                    collisionEvents.add(new CollisionEvent(CollisionEvent.TYPE_PROCESSED,node,node1,cp));
//                else
//                    System.out.println("error finding node during collision");
//        		return true;
//        	}
//    	});
//
//        BulletGlobals.setContactDestroyedCallback(new ContactDestroyedCallback(){
// 			public boolean contactDestroyed(Object userPersistentData) {
//                System.out.println("contact destroyed");
// 				return true;
//			}
//    	});
    }

    private native void nativeUpdate(float time, int maxSteps);

    protected void test(){

    }
    
    /**
     * updates the physics space
     * @param time the current time value
     */
    public void update(float time){
        update(time,1);
    }

    /**
     * updates the physics space, uses maxSteps<br>
     * @param time the current time value
     * @param maxSteps using values bigger than one effectively increases physics speed
     */
    public void update(float time, int maxSteps){
//        if(getDynamicsWorld()==null) return;
        //add recurring events
        rQueue.execute();
        //execute queue
        pQueue.execute();
        //step simulation
        nativeUpdate(time,maxSteps);
        //sync ghostnodes
//        for ( PhysicsGhostNode node : physicsGhostNodes.values() ){
//            node.syncPhysics();
//        }
        //distribute events
        distributeEvents();
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
     * enqueues a Callable in the update queue of the physics thread
     * @param callable the Callable to add
     * @return the created Future for the Callable
     */
    public Future enqueueUpdate(Callable callable){
        return pQueue.enqueue(callable);
    }

    /**
     * enqueues a Callable in the update queue in the next update call
     * (added to avoid loops in update queue)
     * @param callable
     * @return the created Future for the requeue Callable
     */
    public Future reQueue(final Callable callable){
        return rQueue.enqueue(new Callable(){
            public Object call() throws Exception {
                pQueue.enqueue(callable);
                return null;
            }

        });
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
            throw (new UnsupportedOperationException("Cannot remove this kind of object from the physics space."));
        }
    }

    private void addGhostNode(PhysicsGhostNode node){
//        physicsGhostNodes.put(node.getGhostObject(),node);
        if(node instanceof PhysicsCharacterNode){
//            dynamicsWorld.addCollisionObject(node.getGhostObject(), CollisionFilterGroups.CHARACTER_FILTER, (short)(CollisionFilterGroups.STATIC_FILTER | CollisionFilterGroups.DEFAULT_FILTER));
//            getDynamicsWorld().addCollisionObject(node.getGhostObject());
//            dynamicsWorld.addAction(((PhysicsCharacterNode)node).getCharacterController());
        }
        else{
//            getDynamicsWorld().addCollisionObject(node.getGhostObject());
        }
    }

    private void removeGhostNode(PhysicsGhostNode node){
//        physicsGhostNodes.remove(node.getGhostObject());
//        getDynamicsWorld().removeCollisionObject(node.getGhostObject());
//        if(node instanceof PhysicsCharacterNode)
//            dynamicsWorld.removeAction(((PhysicsCharacterNode)node).getCharacterController());
    }

    native void addRigidBody(long number);

    private void addNode(PhysicsNode node){
        physicsNodes.put(node.getRigidBody(),node);
        addRigidBody(node.getRigidBody());
//        getDynamicsWorld().addRigidBody(node.getRigidBody());
//        if(node instanceof PhysicsVehicleNode)
//            dynamicsWorld.addVehicle(((PhysicsVehicleNode)node).getVehicle());
    }

    private void removeNode(PhysicsNode node){
//        physicsNodes.remove(node.getRigidBody());
//        getDynamicsWorld().removeRigidBody(node.getRigidBody());
//        if(node instanceof PhysicsVehicleNode)
//            dynamicsWorld.removeVehicle(((PhysicsVehicleNode)node).getVehicle());
    }

    private void addJoint(PhysicsJoint joint){
        physicsJoints.add(joint);
//        getDynamicsWorld().addConstraint(joint.getConstraint(), !joint.isCollisionBetweenLinkedBodys());
    }

    private void removeJoint(PhysicsJoint joint){
        physicsJoints.remove(joint);
//        getDynamicsWorld().removeConstraint(joint.getConstraint());
    }

    /**
     * sets the gravity of the PhysicsSpace
     * @param gravity
     */
    public void setGravity(Vector3f gravity){
        pSpace.setGravity(gravity);
    }

    /**
     * adds a CollisionListener that will be informed about collision events
     * @param listener the CollisionListener to add
     */
    public void addCollisionListener(CollisionListener listener){
        collisionListeners.add(listener);
    }

    /**
     * removes a CollisionListener from the list
     * @param listener the CollisionListener to remove
     */
    public void removeCollisionListener(CollisionListener listener){
        collisionListeners.remove(listener);
    }

    /**
     * destroys the current PhysicsSpace so that a new one can be created
     */
    public void destroy(){
//        dynamicsWorld.destroy();
//        dynamicsWorld=null;
        pSpace=null;
    }

    /**
     * used internally
     * @return the dynamicsWorld
     */
    public long getDynamicsWorld() {
        return 0;
//        return dynamicsWorld;
    }

    /**
     * interface with Broadphase types
     */
    public interface BroadphaseTypes{
        /**
         * basic Broadphase
         */
        public static final int SIMPLE=0;
        /**
         * better Broadphase, needs worldBounds , max Object number = 16384
         */
        public static final int AXIS_SWEEP_3=1;
        /**
         * better Broadphase, needs worldBounds , max Object number = 65536
         */
        public static final int AXIS_SWEEP_3_32=2;
        public static final int DBVT=3;
        public static final int GPU_3D=4;
    }

}