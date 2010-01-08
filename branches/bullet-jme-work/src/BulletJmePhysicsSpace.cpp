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

#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionDispatch/btGhostObject.h"
#include "BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletMultiThreaded/btGpu3DGridBroadphase.h"

#include "BulletJmePhysicsSpace.h"
#include "BulletJmeMotionState.h"

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
//    m_collisionConfiguration->setConvexConvexMultipointIterations();

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

    broadphase->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
    
    dynWorld->setGravity(btVector3(0, -9.81f, 0));
}

bool BulletJmePhysicsSpace::initJavaMethodHandles(JNIEnv* env) {
    physicsSpaceClass = env->FindClass("com/jmex/bullet/PhysicsSpace");
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return false;
    }

    physicsSpace_test = env->GetMethodID(physicsSpaceClass, "test", "()V");
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return false;
    }

    meshCollisionShapeClass = env->FindClass("com/jmex/bullet/collision/shapes/MeshCollisionShape");
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return false;
    }

    meshCollisionShape_getVerticesArray = env->GetMethodID(meshCollisionShapeClass, "getVerticesArray", "()[F");
    if (env->ExceptionCheck()) {
        env->Throw(env->ExceptionOccurred());
        return false;
    }

    meshCollisionShape_getTrianglesArray = env->GetMethodID(meshCollisionShapeClass, "getTrianglesArray", "()[I");
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

jlong BulletJmePhysicsSpace::createSphereCollisionShape(JNIEnv* env, jobject javaShape, jfloat radius) {
    btSphereShape* shape = new btSphereShape(radius);
    bulletCollisionShapes[collisionShapeIndex]=shape;
    javaCollisionShapes[collisionShapeIndex]=env->NewGlobalRef(javaShape);
    collisionShapeIndex++;
    return collisionShapeIndex-1;
}

jlong BulletJmePhysicsSpace::createCapsuleCollisionShape(JNIEnv* env, jobject javaShape, jfloat radius, jfloat height) {
    btCapsuleShape* shape = new btCapsuleShape(radius,height);
    bulletCollisionShapes[collisionShapeIndex]=shape;
    javaCollisionShapes[collisionShapeIndex]=env->NewGlobalRef(javaShape);
    collisionShapeIndex++;
    return collisionShapeIndex-1;
}

jlong BulletJmePhysicsSpace::createCylinderCollisionShape(JNIEnv* env, jobject javaShape, jfloat extentsX, jfloat extentsY, jfloat extentsZ) {
    btVector3* vector = new btVector3(extentsX, extentsY, extentsZ);
    btCylinderShapeZ* shape = new btCylinderShapeZ(*vector);
    bulletCollisionShapes[collisionShapeIndex]=shape;
    javaCollisionShapes[collisionShapeIndex]=env->NewGlobalRef(javaShape);
    collisionShapeIndex++;
    return collisionShapeIndex-1;
}

jlong BulletJmePhysicsSpace::createMeshCollisionShape(JNIEnv* env, jobject javaShape) {

    jarray tempArray = (jarray)env->CallObjectMethod(javaShape,meshCollisionShape_getVerticesArray);
    int numVertices = env->GetArrayLength(tempArray)/3;
    float* verticesTemp = new float[numVertices*3];
    env->GetFloatArrayRegion((jfloatArray)tempArray,0,numVertices*3,verticesTemp);
    btScalar* vertices = new btScalar[numVertices*3];
    for(int loop=0; loop<numVertices*3; loop++)
    {
            vertices[loop]=(btScalar)verticesTemp[loop];
    }
    free(verticesTemp);
//    env->ReleaseFloatArrayElements((jfloatArray)tempArray,verticesTemp,0);

    tempArray = (jarray)env->CallObjectMethod(javaShape,meshCollisionShape_getTrianglesArray);
    int numTriangles = (env->GetArrayLength(tempArray))/3;
    jint* trianglesTemp = new jint[numTriangles*3];
    env->GetIntArrayRegion((jintArray)tempArray,0,numTriangles*3,trianglesTemp);
    int* triangles = new int[numTriangles*3];
    for(int loop=0; loop<numTriangles*3; loop++)
    {
            triangles[loop]=(int)trianglesTemp[loop];
    }
    free(trianglesTemp);
//    env->ReleaseIntArrayElements((jintArray)tempArray,trianglesTemp,0);
    
    btTriangleIndexVertexArray* meshData = new btTriangleIndexVertexArray(numTriangles,triangles,sizeof(int)*3,numVertices,vertices,sizeof(btScalar)*3);
    btCollisionShape* tempShape;
//    tempShape = new btGImpactMeshShape(meshData);
//    tempShape->setLocalScaling(scale*=worldScale);
//    ((btGImpactMeshShape*)tempShape)->updateBound();
    tempShape = new btBvhTriangleMeshShape(meshData, true, true);

    bulletCollisionShapes[collisionShapeIndex]=tempShape;
    javaCollisionShapes[collisionShapeIndex]=env->NewGlobalRef(javaShape);
    collisionShapeIndex++;
    return collisionShapeIndex-1;
}

jlong BulletJmePhysicsSpace::createRigidBody(JNIEnv* env, jobject javaBody, jlong collisionShapeIndex, jfloat mass) {
    BulletJmeMotionState* motionState = new BulletJmeMotionState(btTransform());
    motionState->setJavaRigidBody(env, javaBody);

    btVector3 localInertia = btVector3(0,0,0);
    if(bulletCollisionShapes[collisionShapeIndex]==NULL){
        fprintf(stdout,"error - no collision shape: %d\n", collisionShapeIndex);
        fflush(stdout);
        return -1;
    }else{
        if(mass>0)
            bulletCollisionShapes[collisionShapeIndex]->calculateLocalInertia(mass,localInertia);
    }

    btRigidBody::btRigidBodyConstructionInfo* constructionInfo
            = new btRigidBody::btRigidBodyConstructionInfo(mass, motionState, bulletCollisionShapes[collisionShapeIndex], localInertia);
    btRigidBody* body=new btRigidBody(*constructionInfo);
    if(mass==0){
        fprintf(stdout,"add static body #%d\n",rigidBodyIndex);
        fflush(stdout);
        body->setCollisionFlags(body->getCollisionFlags() | 1);//CF_STATIC_OBJECT
//        body->setCollisionFlags(body->getCollisionFlags() | 2);//CF_KINEMATIC_OBJECT
//        body->setCollisionFlags(body->getCollisionFlags() & ~4);//CF_NO_CONTACT_RESPONSE
    }
    else{
        fprintf(stdout,"add dynamic body #%d\n",rigidBodyIndex);
        fflush(stdout);
    }

    bulletRigidBodies[rigidBodyIndex]=body;
    javaRigidBodies[rigidBodyIndex]=env->NewGlobalRef(javaBody);
    rigidBodyIndex++;
    return rigidBodyIndex-1;
}

void BulletJmePhysicsSpace::addRigidBody(jlong bodyIndex) {
    dynWorld->addRigidBody(bulletRigidBodies[bodyIndex]);
}

void BulletJmePhysicsSpace::removeRigidBody(jlong bodyIndex) {
    dynWorld->removeRigidBody(bulletRigidBodies[bodyIndex]);
}

void BulletJmePhysicsSpace::translateRigidBody(jlong bodyIndex, jfloat x, jfloat y, jfloat z) {
    //TODO: temp values!
    bulletRigidBodies[bodyIndex]->setWorldTransform(btTransform(btQuaternion(bulletRigidBodies[bodyIndex]->getWorldTransform().getRotation()),btVector3(x,y,z)));
//    bulletRigidBodies[bodyIndex]->getWorldTransform().getOrigin().setValue(x,y,z);
    if(!bulletRigidBodies[bodyIndex]->isStaticOrKinematicObject())
        bulletRigidBodies[bodyIndex]->activate(true);
}

