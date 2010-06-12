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
 * BulletJmeJavaMethods.cpp
 *
 * This file contains all native methods in bullet-jme
 *
 */

#include "com_jmex_bullet_PhysicsSpace.h"
#include "com_jmex_bullet_nodes_PhysicsNode.h"
#include "com_jmex_bullet_collision_shapes_BoxCollisionShape.h"
#include "com_jmex_bullet_collision_shapes_SphereCollisionShape.h"
#include "com_jmex_bullet_collision_shapes_CapsuleCollisionShape.h"
#include "com_jmex_bullet_collision_shapes_CylinderCollisionShape.h"
#include "com_jmex_bullet_collision_shapes_MeshCollisionShape.h"

#include "BulletJmePhysicsSpace.h"

#ifdef __cplusplus
extern "C" {
#endif
    BulletJmePhysicsSpace* g_physSpace = 0;

    /**
     * PhysicsSpace
     */

    JNIEXPORT jboolean JNICALL Java_com_jmex_bullet_PhysicsSpace_startPhysicsSpace(JNIEnv *env, jobject javaBulletPhysicsSpace, 
            jfloat minX, jfloat minY, jfloat minZ, jfloat maxX, jfloat maxY, jfloat maxZ, jint broadphaseType) {

        if(g_physSpace != NULL){
            jclass newExc=env->FindClass("java/lang/IllegalStateException");
            env->ThrowNew(newExc, "The physics space has already been created.");
            return false;
        } else {
            g_physSpace = new BulletJmePhysicsSpace(env,javaBulletPhysicsSpace,minX,minY,minZ,maxX,maxY,maxZ,broadphaseType);
        }

        return true;
    }

    JNIEXPORT void JNICALL Java_com_jmex_bullet_PhysicsSpace_nativeUpdate(JNIEnv *env, jobject javaBulletPhysicsSpace, jfloat time, jint maxSteps) {

        if(g_physSpace != NULL){
            g_physSpace->update(time, maxSteps);
        } else {
            jclass newExc=env->FindClass("java/lang/IllegalStateException");
            env->ThrowNew(newExc, "The physics space has not been created.");
        }

    }

    JNIEXPORT void JNICALL Java_com_jmex_bullet_PhysicsSpace_addRigidBody(JNIEnv *env, jobject javaBulletPhysicsSpace, jlong number) {

        if(g_physSpace != NULL){
            g_physSpace->addRigidBody(number);
        } else {
            jclass newExc=env->FindClass("java/lang/IllegalStateException");
            env->ThrowNew(newExc, "The physics space has not been created.");
        }

    }

    /**
     * CollisionObjects
     */

    JNIEXPORT jlong JNICALL Java_com_jmex_bullet_collision_shapes_BoxCollisionShape_createCollisionBox(JNIEnv* env, jobject javaShape, jfloat extentsX, jfloat extentsY, jfloat extentsZ) {

        if(g_physSpace != NULL){
            return g_physSpace->createBoxCollisionShape(env, javaShape, extentsX, extentsY, extentsZ);
        } else {
            jclass newExc=env->FindClass("java/lang/IllegalStateException");
            env->ThrowNew(newExc, "The physics space has not been created.");
        }
        return -1;
    }

    JNIEXPORT jlong JNICALL Java_com_jmex_bullet_collision_shapes_SphereCollisionShape_createCollisionSphere(JNIEnv* env, jobject javaShape, jfloat radius) {

        if(g_physSpace != NULL){
            return g_physSpace->createSphereCollisionShape(env, javaShape, radius);
        } else {
            jclass newExc=env->FindClass("java/lang/IllegalStateException");
            env->ThrowNew(newExc, "The physics space has not been created.");
        }
        return -1;
    }

    JNIEXPORT jlong JNICALL Java_com_jmex_bullet_collision_shapes_CapsuleCollisionShape_createCollisionCapsule(JNIEnv* env, jobject javaShape, jfloat radius, jfloat height) {

        if(g_physSpace != NULL){
            return g_physSpace->createCapsuleCollisionShape(env, javaShape, radius, height);
        } else {
            jclass newExc=env->FindClass("java/lang/IllegalStateException");
            env->ThrowNew(newExc, "The physics space has not been created.");
        }
        return -1;
    }

    JNIEXPORT jlong JNICALL Java_com_jmex_bullet_collision_shapes_CylinderCollisionShape_createCollisionCylinder(JNIEnv* env, jobject javaShape, jfloat extentsX, jfloat extentsY, jfloat extentsZ) {

        if(g_physSpace != NULL){
            return g_physSpace->createCylinderCollisionShape(env, javaShape, extentsX, extentsY, extentsZ);
        } else {
            jclass newExc=env->FindClass("java/lang/IllegalStateException");
            env->ThrowNew(newExc, "The physics space has not been created.");
        }
        return -1;
    }

    JNIEXPORT jlong JNICALL Java_com_jmex_bullet_collision_shapes_MeshCollisionShape_createCollisionMesh(JNIEnv* env, jobject javaShape) {

        if(g_physSpace != NULL){
            return g_physSpace->createMeshCollisionShape(env, javaShape);
        } else {
            jclass newExc=env->FindClass("java/lang/IllegalStateException");
            env->ThrowNew(newExc, "The physics space has not been created.");
        }
        return -1;
    }

    /**
     * PhysicsNode
     */

    JNIEXPORT jlong JNICALL Java_com_jmex_bullet_nodes_PhysicsNode_createRigidBody(JNIEnv* env, jobject javaBody, jlong collisionShape, jfloat mass) {

        if(g_physSpace != NULL){
            return g_physSpace->createRigidBody(env,javaBody,collisionShape,mass);
        } else {
            jclass newExc=env->FindClass("java/lang/IllegalStateException");
            env->ThrowNew(newExc, "The physics space has not been created.");
        }
        return -1;
    }
	
    JNIEXPORT void JNICALL Java_com_jmex_bullet_nodes_PhysicsNode_setNativeTranslation(JNIEnv* env, jobject javaBody, jlong rigidBody, jfloat x, jfloat y, jfloat z) {

        if(g_physSpace != NULL){
            g_physSpace->translateRigidBody(rigidBody,x,y,z);
        } else {
            jclass newExc=env->FindClass("java/lang/IllegalStateException");
            env->ThrowNew(newExc, "The physics space has not been created.");
        }

    }

    JNIEXPORT void JNICALL Java_com_jmex_bullet_nodes_PhysicsNode_setNativeRotation(JNIEnv* env, jobject javaBody, jlong rigidBody, jfloat m00, jfloat m01, jfloat m02, jfloat m10, jfloat m11, jfloat m12, jfloat m20, jfloat m21, jfloat m22) {

        if(g_physSpace != NULL){
            g_physSpace->rotateRigidBody(rigidBody,m00,m01,m02,m10,m11,m12,m20,m21,m22);
        } else {
            jclass newExc=env->FindClass("java/lang/IllegalStateException");
            env->ThrowNew(newExc, "The physics space has not been created.");
        }

    }

    JNIEXPORT void JNICALL Java_com_jmex_bullet_nodes_PhysicsNode_setNativeGravity(JNIEnv* env, jobject javaBody, jlong rigidBody, jfloat x, jfloat y, jfloat z) {

        if(g_physSpace != NULL){
            g_physSpace->setRigidBodyGravity(rigidBody,x,y,z);
        } else {
            jclass newExc=env->FindClass("java/lang/IllegalStateException");
            env->ThrowNew(newExc, "The physics space has not been created.");
        }

    }

    JNIEXPORT void JNICALL Java_com_jmex_bullet_nodes_PhysicsNode_setNativeFriction(JNIEnv* env, jobject javaBody, jlong rigidBody, jfloat friction) {

        if(g_physSpace != NULL){
            g_physSpace->setRigidBodyFriction(rigidBody,friction);
        } else {
            jclass newExc=env->FindClass("java/lang/IllegalStateException");
            env->ThrowNew(newExc, "The physics space has not been created.");
        }

    }

    JNIEXPORT void JNICALL Java_com_jmex_bullet_nodes_PhysicsNode_setNativeDamping(JNIEnv* env, jobject javaBody, jlong rigidBody, jfloat linear, jfloat angular) {

        if(g_physSpace != NULL){
            g_physSpace->setRigidBodyDamping(rigidBody,linear, angular);
        } else {
            jclass newExc=env->FindClass("java/lang/IllegalStateException");
            env->ThrowNew(newExc, "The physics space has not been created.");
        }

    }

    JNIEXPORT void JNICALL Java_com_jmex_bullet_nodes_PhysicsNode_setNativeRestitution(JNIEnv* env, jobject javaBody, jlong rigidBody, jfloat restitution) {

        if(g_physSpace != NULL){
            g_physSpace->setRigidBodyRestitution(rigidBody,restitution);
        } else {
            jclass newExc=env->FindClass("java/lang/IllegalStateException");
            env->ThrowNew(newExc, "The physics space has not been created.");
        }

    }

#ifdef __cplusplus
}
#endif