/*
 * Copyright (c) 2010 Normen Hansen
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
#include "BulletJmeMotionState.h"

BulletJmeMotionState::BulletJmeMotionState(btTransform worldTrans) {
//    fprintf(stdout,"new motionstate\n");
    worldTransform=worldTrans;
    dirty=true;
}

void BulletJmeMotionState::getWorldTransform(btTransform& worldTrans ) const{
    worldTrans=worldTransform;
//    fprintf(stdout,"Get world transform\n");
}

void BulletJmeMotionState::setWorldTransform(const btTransform& worldTrans){
    worldTransform=worldTrans;
    dirty=true;
    float x=worldTransform.getOrigin().m_floats[0];
    if(env!=NULL && javaRigidBody!=NULL){
//        jvm->AttachCurrentThread((void**)&env, NULL);

        env->CallVoidMethod(this->javaRigidBody,method_physicsNode_setWorldTranslation,
                worldTransform.getOrigin().m_floats[0],
                worldTransform.getOrigin().m_floats[1],
                worldTransform.getOrigin().m_floats[2]);
        if (env->ExceptionCheck()) env->Throw(env->ExceptionOccurred());

        env->CallVoidMethod(this->javaRigidBody,method_physicsNode_setWorldRotation,
                worldTransform.getBasis().getColumn(0).m_floats[0],
                worldTransform.getBasis().getColumn(0).m_floats[1],
                worldTransform.getBasis().getColumn(0).m_floats[2],
                worldTransform.getBasis().getColumn(1).m_floats[0],
                worldTransform.getBasis().getColumn(1).m_floats[1],
                worldTransform.getBasis().getColumn(1).m_floats[2],
                worldTransform.getBasis().getColumn(2).m_floats[0],
                worldTransform.getBasis().getColumn(2).m_floats[1],
                worldTransform.getBasis().getColumn(2).m_floats[2]);
        if (env->ExceptionCheck()) env->Throw(env->ExceptionOccurred());
    }
}

jobject BulletJmeMotionState::getJavaRigidBody(){
    return this->javaRigidBody;
}

void BulletJmeMotionState::setJavaRigidBody(JNIEnv* env, jobject javaRigidBody){
    initJavaMethodHandles(env);
    this->env=env;
    this->javaRigidBody=env->NewGlobalRef(javaRigidBody);
}

bool BulletJmeMotionState::initJavaMethodHandles(JNIEnv* env) {
//    env->GetJavaVM(&jvm);

    physicsNodeClass = env->FindClass("com/jmex/bullet/nodes/PhysicsNode");
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return false;
    }

    method_physicsNode_setWorldTranslation = env->GetMethodID(physicsNodeClass, "setWorldTranslation", "(FFF)V");
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return false;
    }

    method_physicsNode_setWorldRotation = env->GetMethodID(physicsNodeClass, "setWorldRotation", "(FFFFFFFFF)V");
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return false;
    }

    return true;
}

BulletJmeMotionState::~BulletJmeMotionState() {
    // TODO Auto-generated destructor stub
}
