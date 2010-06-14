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
package com.jmex.jbullet.debug;

import com.bulletphysics.collision.shapes.ConcaveShape;
import com.bulletphysics.collision.shapes.TriangleCallback;
import com.jme.renderer.Renderer;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import javax.vecmath.Vector3f;

/**
 *  The rigid body wireframe for a concave shape.
 *
 * @author CJ Hare
 */
public class ConcaveWireframe extends TriMeshWireframe implements RigidBodyWireframe
{
    /** The maximum corner for the aabb used for triangles to include in ConcaveShape processing.*/
    private static final Vector3f aabbMax = new Vector3f( 1e30f, 1e30f, 1e30f );

    /** The minimum corner for the aabb used for triangles to include in ConcaveShape processing.*/
    private static final Vector3f aabbMin = new Vector3f( -1e30f, -1e30f, -1e30f );

    /**
     *  Creates a wireframe object based on the given Concave Shape.
     *
     * @param concaveShape the shape that'll determine what the wireframe object will look like.
     * @return the wireframe Spatial that can be manipulated just like any other!
     */
    public ConcaveWireframe( ConcaveShape concaveShape )
    {
        createRigidBodyWireframe( getVertices( concaveShape ) );
    }

    /**
     *  Constructs the buffer for the vertices of the concave shape.
     *
     * @param concaveShape the shape to get the vertices for / from.
     * @return the shape as stored by the given broadphase rigid body.
     */
    private FloatBuffer getVertices( ConcaveShape concaveShape )
    {
        // Create the call back that'll create the vertex buffer
        BufferedTriangleCallback triangleProcessor = new BufferedTriangleCallback();
        concaveShape.processAllTriangles( triangleProcessor, aabbMin, aabbMax );

        // Retrieve the vextex and index buffers
        return triangleProcessor.getVertices();
    }

    @Override
    public void render( Renderer renderer )
    {
        renderer.draw( this );
    }
}

/**
 *  A callback is used to process the triangles of the shape as there is no direct access to a concave shapes, shape.
 *  <p/>
 *  The triangles are simply put into a list (which in extreme condition will cause memory problems) then put into a direct buffer.
 *
 * @author CJ Hare
 */
class BufferedTriangleCallback implements TriangleCallback
{
    private ArrayList<Vector3f> vertices;

    public BufferedTriangleCallback()
    {
        vertices = new ArrayList<Vector3f>();
    }

    @Override
    public void processTriangle( Vector3f[] triangle, int partId, int triangleIndex )
    {
        // Three sets of individual lines
        // The new Vector is needed as the given triangle reference is from a pool
        vertices.add( new Vector3f( triangle[0] ) );
        vertices.add( new Vector3f( triangle[1] ) );
        vertices.add( new Vector3f( triangle[2] ) );
    }

    /**
     *  Retrieves the vertices from the Triangle buffer.
     */
    public FloatBuffer getVertices()
    {
        // There are 3 floats needed for each vertex (x,y,z)
        final int numberOfFloats = vertices.size() * 3;
        final int byteBufferSize = numberOfFloats * Float.SIZE;
        FloatBuffer verticesBuffer = ByteBuffer.allocateDirect( byteBufferSize ).order( ByteOrder.nativeOrder() ).asFloatBuffer();

        // Force the limit, set the cap - most number of floats we will use the buffer for
        verticesBuffer.limit( numberOfFloats );

        // Copy the values from the list to the direct float buffer
        for ( Vector3f v : vertices )
        {
            verticesBuffer.put( v.x ).put( v.y ).put( v.z );
        }

        return verticesBuffer;
    }
}

