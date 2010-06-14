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

import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.collision.shapes.ShapeHull;
import com.bulletphysics.util.IntArrayList;
import com.jme.renderer.Renderer;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.List;
import javax.vecmath.Vector3f;

/**
 *  The rigid body wireframe for a convex shape.
 * 
 * @author CJ Hare
 */
public class ConvexWireframe extends TriMeshWireframe implements RigidBodyWireframe
{
    /**
     *  Creates a wireframe from a Convex shape.
     *
     * @param convexShape the shape that the wireframe will be created from, define what shape the wireframe will take.
     * @return the wireframe Spatial that can be manipulated just like any other!
     */
    public ConvexWireframe( ConvexShape convexShape )
    {
        createRigidBodyWireframe( getVertices( convexShape ) );
    }

    /**
     *  Processes the given convex shape to retrieve a correctly ordered FloatBuffer to
     *  construct the shape from with a TriMesh.
     *
     * @param convexShape the shape to retreieve the vertices from.
     * @return the vertices as a FloatBuffer, ordered as Triangles.
     */
    private FloatBuffer getVertices( ConvexShape convexShape )
    {
        // Check there is a hull shape to render
        if ( convexShape.getUserPointer() == null )
        {
            // create a hull approximation
            ShapeHull hull = new ShapeHull( convexShape );
            float margin = convexShape.getMargin();
            hull.buildHull( margin );
            convexShape.setUserPointer( hull );
        }

        // Assert state - should have a pointer to a hull (shape) that'll be drawn
        assert convexShape.getUserPointer() != null : "Should have a shape for the userPointer, instead got null";
        ShapeHull hull = (ShapeHull) convexShape.getUserPointer();

        // Assert we actually have a shape to render
        assert hull.numTriangles() > 0 : "Expecting the Hull shape to have triangles";
        int numberOfTriangles = hull.numTriangles();

        // The number of bytes needed is: (floats in a vertex) * (vertices in a triangle) * (# of triangles) * (size of float in bytes)
        final int numberOfFloats = 3 * 3 * numberOfTriangles;
        final int byteBufferSize = numberOfFloats * Float.SIZE;
        FloatBuffer vertices = ByteBuffer.allocateDirect( byteBufferSize ).order( ByteOrder.nativeOrder() ).asFloatBuffer();

        // Force the limit, set the cap - most number of floats we will use the buffer for
        vertices.limit( numberOfFloats );

        // Loop variables
        final IntArrayList hullIndicies = hull.getIndexPointer();
        final List<Vector3f> hullVertices = hull.getVertexPointer();
        Vector3f vertexA, vertexB, vertexC;
        int index = 0;

        for ( int i = 0; i < numberOfTriangles; i++ )
        {
            // Grab the data for this triangle from the hull
            vertexA = hullVertices.get( hullIndicies.get( index++ ) );
            vertexB = hullVertices.get( hullIndicies.get( index++ ) );
            vertexC = hullVertices.get( hullIndicies.get( index++ ) );

            // Put the verticies into the vertex buffer
            vertices.put( vertexA.x ).put( vertexA.y ).put( vertexA.z );
            vertices.put( vertexB.x ).put( vertexB.y ).put( vertexB.z );
            vertices.put( vertexC.x ).put( vertexC.y ).put( vertexC.z );
        }

        return vertices;
    }

    @Override
    public void render( Renderer renderer )
    {
        renderer.draw( this );
    }
}
