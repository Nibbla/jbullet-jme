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
package com.jmex.jbullet.nodes.infos;

import com.bulletphysics.linearmath.MotionState;
import com.bulletphysics.linearmath.Transform;
import com.g3d.math.Matrix3f;
import com.g3d.math.Quaternion;
import com.g3d.math.Vector3f;
import com.g3d.scene.Spatial;
import com.jmex.jbullet.nodes.PhysicsNode;
import com.jmex.jbullet.util.Converter;

/**
 * stores all info about a bullet physics object in a thread-safe manner to
 * allow access from the jme scenegraph and the bullet physicsspace<br>
 * @author normenhansen
 */
public class PhysicsNodeState implements MotionState{
    //stores the bullet transform
    private Transform motionStateTrans=new Transform();
    
    //stores jme transform info
    private Vector3f worldLocation=new Vector3f();
    private Matrix3f worldRotation=new Matrix3f();
    private Quaternion worldRotationQuat=new Quaternion();
    //keep track of transform changes
    private boolean physicsLocationDirty=false;
    private boolean jmeLocationDirty=true;
    private boolean jmeVelocityDirty=false;

    private boolean physicsStop=false;
    //temp variable for conversion
    private Quaternion tmp_inverseWorldRotation=new Quaternion();

    private Vector3f linearVelocity=new Vector3f();
    private Vector3f angularVelocity=new Vector3f();

    private PhysicsNode pNode;

    public PhysicsNodeState(PhysicsNode pNode) {
        this.pNode = pNode;
    }

    public synchronized Transform getWorldTransform(Transform out) {
        //TODO: check if really needed
        if(!isJmeLocationDirty()) return out;
        if(out==null)
            out=new Transform();
        out.set(motionStateTrans);
//        System.out.println("get worldtransform from physics");
        setJmeLocationDirty(false);
        return out;
    }

    public synchronized void setWorldTransform(Transform worldTrans) {
        if(jmeLocationDirty) return;
//        System.out.println("set worldtransform from physics");
        motionStateTrans.set(worldTrans);
        Converter.convert(worldTrans.origin, worldLocation);
        Converter.convert(worldTrans.basis, worldRotation);
        worldRotationQuat.fromRotationMatrix(worldRotation);
        physicsLocationDirty=true;
    }

    public synchronized boolean getWorldTransform(Vector3f location, Quaternion rot){
        if(!physicsLocationDirty) return false;
//        System.out.println("get worldtransform from jme");
        location.set(worldLocation);
        rot.set(worldRotationQuat);
        physicsLocationDirty=false;
        return true;
    }

    public synchronized boolean getLocalTransform(Spatial parent, Vector3f location, Quaternion rotation){
        if(!physicsLocationDirty) return false;
//        System.out.println("get worldtransform from jme");
        location.set(worldLocation);
        rotation.set(worldRotationQuat);
        location.subtractLocal(parent.getWorldTranslation());
        location.divideLocal( parent.getWorldScale() );
        tmp_inverseWorldRotation.set( parent.getWorldRotation()).inverseLocal().multLocal( location );
        tmp_inverseWorldRotation.set( parent.getWorldRotation()).inverseLocal().mult( worldRotationQuat, rotation );
        physicsLocationDirty=false;
        return true;
    }

    public synchronized void setWorldTransform(Vector3f location, Quaternion rotation){
//        System.out.println("set worldtransform from jme");
        worldLocation.set(location);
        worldRotationQuat.set(rotation);
        worldRotation.set(rotation.toRotationMatrix());
        Converter.convert(worldLocation,motionStateTrans.origin);
        Converter.convert(worldRotation,motionStateTrans.basis);
        setJmeLocationDirty(true);
    }

    public synchronized boolean isJmeLocationDirty() {
        return jmeLocationDirty;
    }

    public synchronized void setJmeLocationDirty(boolean jmeDirty) {
        this.jmeLocationDirty = jmeDirty;
    }

    public synchronized Vector3f getLinearVelocity() {
        return linearVelocity;
    }

    public synchronized void setLinearVelocity(Vector3f linearVelocity) {
        jmeVelocityDirty=true;
        this.linearVelocity = linearVelocity;
    }

    public synchronized Vector3f getAngularVelocity() {
        return angularVelocity;
    }

    public synchronized void setAngularVelocity(Vector3f angularVelocity) {
        jmeVelocityDirty=true;
        this.angularVelocity = angularVelocity;
    }

    public synchronized boolean isPhysicsStop() {
        return physicsStop;
    }

    public synchronized void setPhysicsStop(boolean physicsStop) {
        this.physicsStop = physicsStop;
    }

}
