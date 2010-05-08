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

import com.jme.bounding.BoundingBox;
import com.jme.light.DirectionalLight;
import com.jme.math.Quaternion;
import com.jme.renderer.ColorRGBA;
import com.jme.scene.Spatial;
import com.jme.scene.TriMesh;
import com.jme.scene.VBOInfo;
import com.jme.scene.state.LightState;
import com.jme.scene.state.MaterialState;
import com.jme.scene.state.WireframeState;
import com.jme.scene.state.ZBufferState;
import com.jme.system.DisplaySystem;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import javax.vecmath.Vector3f;

/**
 *  A customised version of TriMesh for use with the PhysicsDebug view.
 * <p/>
 *  The aim is to provide encapsulation for the creation of a wireframe from a bullet physics
 *  broadphase rigid body, managing the translation/rotation and the rendering using the standard
 *  JME renderer.
 *
 * @author CJ Hare
 */
public abstract class TriMeshWireframe extends TriMesh
{
    /** The smallest float increment to deal with, for translation && rotations.*/
    private static final float SMALLEST_FLOAT_CHANGE = 0.001f;

    /** The standard ordering of objects by their z order, keeps things correctly in perspective.*/
    private static final ZBufferState zOrdered;

    /** The wireframe state shared that makes the renderer draw wireframes, not fill in the triangles.*/
    private static final WireframeState wireframeState;

    /** The single light state applied to all the physics bounds meshes, get the MaterialState to work (saves on geometry colours)*/
    private static final LightState lighting;

    static
    {
        zOrdered = DisplaySystem.getDisplaySystem().getRenderer().createZBufferState();
        zOrdered.setWritable( true );
        zOrdered.setEnabled( true );
        zOrdered.setFunction( ZBufferState.TestFunction.LessThanOrEqualTo );

        wireframeState = DisplaySystem.getDisplaySystem().getRenderer().createWireframeState();
        wireframeState.setEnabled( true );
        wireframeState.setLineWidth( 1f );
        wireframeState.setAntialiased( false );

        lighting = DisplaySystem.getDisplaySystem().getRenderer().createLightState();
        lighting.setGlobalAmbient( ColorRGBA.white );
        DirectionalLight light = new DirectionalLight();
        light.setDirection( new com.jme.math.Vector3f( 0f, 1f, 0f ) );
        light.setAmbient( ColorRGBA.white );
        light.setDiffuse( ColorRGBA.white );
        lighting.attach( light );
    }

    /**
     *  The available bullet states that rigid body can be in at any time.
     */
    private enum ActivityState
    {
        //TODO find out all the possible states
        A( ColorRGBA.red ), B( ColorRGBA.blue );

        public final MaterialState colour;

        ActivityState( ColorRGBA stateColour )
        {
            colour = DisplaySystem.getDisplaySystem().getRenderer().createMaterialState();
            colour.setDiffuse( stateColour );
            colour.setAmbient( stateColour );
            colour.setEmissive( stateColour );
            colour.setSpecular( stateColour );
            colour.setEnabled( true );
            colour.setMaterialFace( MaterialState.MaterialFace.FrontAndBack );
        }
    }

    /**
     *  Creates a TriMesh setting it up so that it is rendered as a wireframe.
     *
     * @param vertices the vextex buffer that the index buffer refers to.
     * @return The create Spatial that is a wireframe with the shared states of all PhysicsDebugger wireframe objects.
     */
    public void createRigidBodyWireframe( FloatBuffer vertices )
    {
        setVertexBuffer( vertices );
        setIndexBuffer( generateIndicies( vertices ) );

        // Create the model bounds for frustum culling
        setModelBound( new BoundingBox() );
        updateModelBound();

        // Cull only when not inside || intersecting with frustum
        setCullHint( Spatial.CullHint.Dynamic );

        // TODO default state?
        // The default bullet state when objects are added is: ????
        setRenderState( ActivityState.A.colour );

        // Apply the shared state for wireframe, drawing order and starting colour
        setRenderState( wireframeState );
        setRenderState( zOrdered );
        setRenderState( lighting );

        // The render state needs updating to pickup the change in colour
        updateRenderState();

        // Use Vertex Buffering to achieve substainally better rendering with high poly objects
        setVBOInfo( new VBOInfo( true ) );
    }

    /**
     *  Generates a normally ordered index buffer for the given number of triangles.<br/>
     *  Basically when a array of vertex are ordered where each three vertex make a triangle generateIndicies()
     *  will generate the correct index to match (needed for TriMesh).
     *
     * @param vertices the vertex buffer, for which the index buffer is being created.
     * @return the normally ordered index buffer (0,1,2,3,4....etc)
     */
    private IntBuffer generateIndicies( FloatBuffer vertices )
    {
        // Number of indices needed: (3 vertex in triangle) * (# of triangles)
        final int numberOfIndicies = vertices.limit() / 3;
        IntBuffer indicies = ByteBuffer.allocateDirect( numberOfIndicies * Integer.SIZE ).order( ByteOrder.nativeOrder() ).asIntBuffer();

        // Restrict the potential size of the buffer
        indicies.limit( numberOfIndicies );

        for ( int i = 0; i < numberOfIndicies; i++ )
        {
            indicies.put( i );
        }

        return indicies;
    }

    /**
     *  Ensures the wireframe's world translation matches that given.
     *
     * @param bulletTranslation the coordinates of the wireframe's paired broadphase rigid body from the bullet physics simulation.
     */
    public void updateWorldTranslation( Vector3f bulletTranslation )
    {
        // Check if the worldTranslation of the object has changed
        if ( isNotEqual( bulletTranslation, worldTranslation ) )
        {
            // The physics object has moved since last render - update the world translation of the Spatial
            setLocalTranslation( bulletTranslation.x, bulletTranslation.y, bulletTranslation.z );
            updateGeometricState( 0f, true );
        }
    }

    /**
     * Ensures the wireframe's world rotation matches that given.
     *
     * @param bulletRotation the rotation of the wireframe's paired broadphase rigid body from the bullet physics simulation.
     */
    public void updateWorldRotation( Quaternion bulletRotation )
    {
        // Check if the worldRotation of the object has changed
        if ( isNotEqual( bulletRotation, worldRotation ) )
        {
            // The physics object has rotated since last render - update the world rotation of the Spatial
            getLocalRotation().set( bulletRotation );
            updateGeometricState( 0f, true );
        }
    }

    /**
     * Is there equality between the values held by the Quaternions?
     * <p/>
     *  The measure of equality for this inline method is whether there is a difference of greater then
     * SMALLEST_FLOAT_CHANGE between the two floats.
     *
     * @param a the first Quaternion.
     * @param b the second Quaternion.
     * @return <code>true</code> the difference in value between the vectors values is greater then acceptable,
     *      <code>false</code> otherwise.
     */
    private final boolean isNotEqual( Quaternion a, Quaternion b )
    {
        return isNotEqual( a.x, b.x )
                || isNotEqual( a.y, b.y )
                || isNotEqual( a.z, b.z )
                || isNotEqual( a.w, b.w );
    }

    /**
     *  Is there equality between the values held by the vectors?
     * <p/>
     *  The measure of equality for this inline method is whether there is a difference of greater then
     * SMALLEST_FLOAT_CHANGE between the two floats.
     *
     * @param a the first vector.
     * @param b the second vector.
     * @return <code>true</code> the difference in value between the vectors values is greater then acceptable,
     *      <code>false</code> otherwise.
     */
    private final boolean isNotEqual( Vector3f a, com.jme.math.Vector3f b )
    {
        return isNotEqual( a.x, b.x )
                || isNotEqual( a.y, b.y )
                || isNotEqual( a.z, b.z );
    }

    /**
     *  Are the two floats considered equal?
     * <p/>
     *  The measure of equality for this inline method is whether there is a difference of greater then
     * SMALLEST_FLOAT_CHANGE between the two floats.
     *
     * @param a the first float.
     * @param b the second float.
     * @return <code>true</code> the difference in value between the floats is greater then acceptable,
     *      <code>false</code> otherwise.
     */
    private final boolean isNotEqual( float a, float b )
    {
        return a > b + SMALLEST_FLOAT_CHANGE || b > a + SMALLEST_FLOAT_CHANGE;
    }
}
