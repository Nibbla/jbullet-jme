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

/**
 * TODO: a lot!
 * - use arrays instad of jfloats for native access.. (MotionState etc)
 */

#include "BulletJmePhysicsSpace.h"

BulletJmePhysicsSpace::BulletJmePhysicsSpace(JNIEnv* env, jobject physicsSpace,
        jfloat minX, jfloat minY, jfloat minZ, jfloat maxX, jfloat maxY, jfloat maxZ, jint broadphaseType) {
    this->javaPhysicsSpace = env->NewGlobalRef(physicsSpace);
    rigidBodyIndex = 0;
    collisionShapeIndex = 0;
    constraintIndex = 0;
//    this->tempTrans=btTransform(btQuaternion(0,0,0,0),btVector3(0,0,0));

    if (!initJavaMethodHandles(env)) {
        free(this);
        return;
    }

    // collision configuration contains default setup for memory, collision setup
    btCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
    //m_collisionConfiguration->setConvexConvexMultipointIterations();

    // use the default collision dispatcher. For parallel processing you can use a diffent dispatcher (see Extras/BulletMultiThreaded)
    btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collisionConfiguration);

    btVector3 min = btVector3(minX, minY, minZ);
    btVector3 max = btVector3(maxX, maxY, maxZ);

    btBroadphaseInterface* broadphase;

    switch (broadphaseType) {
        case 0:
            broadphase = new btSimpleBroadphase();
            break;
        case 1:
            broadphase = new btAxisSweep3(min, max);
            break;
        case 2:
            //TODO: 32bit!
            broadphase = new btAxisSweep3(min, max);
            break;
        case 3:
            broadphase = new btDbvtBroadphase();
            break;
        case 4:
            broadphase = new btGpu3DGridBroadphase(
                    min, max,
                    20, 20, 20,
                    10000, 1000, 25);
            break;
    }

    // the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    btConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

    dynWorld = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);

    dynWorld->setGravity(btVector3(0, -9.81f, 0));
}

bool BulletJmePhysicsSpace::initJavaMethodHandles(JNIEnv* env) {
    physicsSpaceClass = env->FindClass("com/jmex/bullet/PhysicsSpace");
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return false;
    }

    method_physicsSpace_test = env->GetMethodID(physicsSpaceClass, "test", "()V");
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return false;
    }
    return true;
}

void BulletJmePhysicsSpace::update(jfloat timestep, jint maxSteps) {
    dynWorld->stepSimulation(timestep, maxSteps);
}

jlong BulletJmePhysicsSpace::createBoxCollisionShape(JNIEnv* env, jobject javaShape, jfloat extentsX, jfloat extentsY, jfloat extentsZ) {
    btVector3* vector = new btVector3(extentsX, extentsY, extentsZ);
    btBoxShape* shape = new btBoxShape(*vector);
    bulletCollisionShapes[collisionShapeIndex]=shape;
    javaCollisionShapes[collisionShapeIndex]=env->NewGlobalRef(javaShape);
    collisionShapeIndex++;
    return collisionShapeIndex-1;
}

jlong BulletJmePhysicsSpace::createRigidBody(JNIEnv* env, jobject javaBody, jlong collisionShapeIndex, jfloat mass) {
    BulletJmeMotionState* ms = new BulletJmeMotionState(btTransform());
    ms->setJavaRigidBody(env, javaBody);
    
    btVector3 localInertia = btVector3();
    btRigidBody::btRigidBodyConstructionInfo* constructionInfo 
            = new btRigidBody::btRigidBodyConstructionInfo(mass, ms, bulletCollisionShapes[collisionShapeIndex], localInertia);
    
    btRigidBody* body=new btRigidBody(*constructionInfo);
    if(mass==0){
        body->setCollisionFlags(body->getCollisionFlags() | 1);//CF_STATIC_OBJECT
    }
    if(bulletCollisionShapes[collisionShapeIndex]==NULL){
        fprintf(stdout,"no shape\n");
        fflush(stdout);
    }else{
        fprintf(stdout,"HAS SHAPE!\n");
        fflush(stdout);
    }
    bulletRigidBodies[rigidBodyIndex]=body;
    javaRigidBodies[rigidBodyIndex]=env->NewGlobalRef(javaBody);
    rigidBodyIndex++;
    return rigidBodyIndex-1;
}

void BulletJmePhysicsSpace::removeRigidBody(jlong bodyIndex) {
    dynWorld->removeRigidBody(bulletRigidBodies[bodyIndex]);
}

void BulletJmePhysicsSpace::addRigidBody(jlong bodyIndex) {
    dynWorld->addRigidBody(bulletRigidBodies[bodyIndex]);
}

void BulletJmePhysicsSpace::translateRigidBody(jlong bodyIndex, jfloat x, jfloat y, jfloat z) {
    bulletRigidBodies[bodyIndex]->getWorldTransform().getOrigin().setValue(x,y,z);
    if(!bulletRigidBodies[bodyIndex]->isStaticOrKinematicObject())
                bulletRigidBodies[bodyIndex]->activate(true);
}

