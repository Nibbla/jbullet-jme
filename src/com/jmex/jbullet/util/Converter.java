/*
 * Copyright (c) 2005-2008 jME Physics 2
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 *  * Neither the name of 'jME Physics 2' nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
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
package com.jmex.jbullet.util;

import java.nio.ByteBuffer;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;

/**
 * Nice convenience methods for conversion between javax.vecmath and com.jme.math
 * Objects, also some jme to jbullet mesh conversion. Mostly taken from the
 * jmePhysics2-jbullet library (http://jmephysics.googlecode.com).
 * @author normenhansen
 */
public class Converter {

    private Converter() {
    }

    public static com.jme.math.Vector3f convert( javax.vecmath.Vector3f oldVec ) {
        com.jme.math.Vector3f newVec = new com.jme.math.Vector3f();
        convert( oldVec, newVec );
        return newVec;
    }

    public static void convert( javax.vecmath.Vector3f oldVec, com.jme.math.Vector3f newVec ) {
        newVec.x = oldVec.x;
        newVec.y = oldVec.y;
        newVec.z = oldVec.z;
    }

    public static javax.vecmath.Vector3f convert( com.jme.math.Vector3f oldVec ) {
        javax.vecmath.Vector3f newVec = new javax.vecmath.Vector3f();
        convert( oldVec, newVec );
        return newVec;
    }

    public static void convert( com.jme.math.Vector3f oldVec, javax.vecmath.Vector3f newVec ) {
        newVec.x = oldVec.x;
        newVec.y = oldVec.y;
        newVec.z = oldVec.z;
    }

    public static void convert( com.jme.math.Quaternion oldQuat, javax.vecmath.Quat4f newQuat ) {
        newQuat.w = oldQuat.w;
        newQuat.x = oldQuat.x;
        newQuat.y = oldQuat.y;
        newQuat.z = oldQuat.z;
    }

    public static javax.vecmath.Quat4f convert( com.jme.math.Quaternion oldQuat ) {
        javax.vecmath.Quat4f newQuat = new javax.vecmath.Quat4f();
        convert( oldQuat, newQuat );
        return newQuat;
    }

    public static void convert( javax.vecmath.Quat4f oldQuat, com.jme.math.Quaternion newQuat ) {
        newQuat.w = oldQuat.w;
        newQuat.x = oldQuat.x;
        newQuat.y = oldQuat.y;
        newQuat.z = oldQuat.z;
    }

    public static com.jme.math.Quaternion convert( javax.vecmath.Quat4f oldQuat ) {
        com.jme.math.Quaternion newQuat = new com.jme.math.Quaternion();
        convert( oldQuat, newQuat );
        return newQuat;
    }

    public static com.jme.math.Matrix3f convert( javax.vecmath.Matrix3f oldMatrix ) {
        com.jme.math.Matrix3f newMatrix = new com.jme.math.Matrix3f();
        convert( oldMatrix, newMatrix );
        return newMatrix;
    }

    public static void convert( javax.vecmath.Matrix3f oldMatrix, com.jme.math.Matrix3f newMatrix ) {
        newMatrix.m00 = oldMatrix.m00;
        newMatrix.m01 = oldMatrix.m01;
        newMatrix.m02 = oldMatrix.m02;
        newMatrix.m10 = oldMatrix.m10;
        newMatrix.m11 = oldMatrix.m11;
        newMatrix.m12 = oldMatrix.m12;
        newMatrix.m20 = oldMatrix.m20;
        newMatrix.m21 = oldMatrix.m21;
        newMatrix.m22 = oldMatrix.m22;
    }

    public static javax.vecmath.Matrix3f convert( com.jme.math.Matrix3f oldMatrix ) {
        javax.vecmath.Matrix3f newMatrix = new javax.vecmath.Matrix3f();
        convert( oldMatrix, newMatrix );
        return newMatrix;
    }

    public static void convert( com.jme.math.Matrix3f oldMatrix, javax.vecmath.Matrix3f newMatrix ) {
        newMatrix.m00 = oldMatrix.m00;
        newMatrix.m01 = oldMatrix.m01;
        newMatrix.m02 = oldMatrix.m02;
        newMatrix.m10 = oldMatrix.m10;
        newMatrix.m11 = oldMatrix.m11;
        newMatrix.m12 = oldMatrix.m12;
        newMatrix.m20 = oldMatrix.m20;
        newMatrix.m21 = oldMatrix.m21;
        newMatrix.m22 = oldMatrix.m22;
    }

    public static com.bulletphysics.collision.shapes.TriangleIndexVertexArray convert( com.jme.scene.TriMesh mesh ) {
        com.bulletphysics.collision.shapes.TriangleIndexVertexArray jBulletMeshData
                = new com.bulletphysics.collision.shapes.TriangleIndexVertexArray();

        com.bulletphysics.collision.shapes.IndexedMesh jBulletIndexedMesh
                = new com.bulletphysics.collision.shapes.IndexedMesh();
        jBulletIndexedMesh.triangleIndexBase = ByteBuffer.allocate( mesh.getTriangleCount() * 3 * 4 );
        jBulletIndexedMesh.vertexBase = ByteBuffer.allocate( mesh.getVertexCount() * 3 * 4 );

        IntBuffer indices = mesh.getIndexBuffer();
        indices.rewind();

        FloatBuffer vertices = mesh.getVertexBuffer();
        vertices.rewind();

        int verticesLength = mesh.getVertexCount() * 3;
        jBulletIndexedMesh.numVertices = mesh.getVertexCount();
        jBulletIndexedMesh.vertexStride = 12; //3 verts * 4 bytes per.
        for ( int i = 0; i < verticesLength; i++ ) {
            float tempFloat = vertices.get();
            jBulletIndexedMesh.vertexBase.putFloat( tempFloat );
        }

        int indicesLength = mesh.getTriangleCount() * 3;
        jBulletIndexedMesh.numTriangles = mesh.getTriangleCount();
        jBulletIndexedMesh.triangleIndexStride = 12; //3 index entries * 4 bytes each.
        for ( int i = 0; i < indicesLength; i++ ) {
            jBulletIndexedMesh.triangleIndexBase.putInt( indices.get() );
        }

        jBulletMeshData.addIndexedMesh( jBulletIndexedMesh );

        return jBulletMeshData;
    }
}
