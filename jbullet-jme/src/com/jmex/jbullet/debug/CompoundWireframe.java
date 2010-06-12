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

import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.CompoundShape;
import com.bulletphysics.collision.shapes.CompoundShapeChild;
import com.bulletphysics.collision.shapes.ConcaveShape;
import com.bulletphysics.collision.shapes.ConvexShape;
import com.jme.bounding.BoundingBox;
import com.jme.math.Quaternion;
import com.jme.renderer.Renderer;
import com.jme.scene.Node;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import javax.vecmath.Matrix3f;
import javax.vecmath.Vector3f;

/**
 *  The rigid body wireframe for a compund shape.
 * <p/>
 *  The compound shape pairs with the description for the btCompoundShape,
 *  chiefly that it is comprised of other primatives (see bullet manual P16 - CollisionShapes).
 *
 * @author CJ Hare
 */
public class CompoundWireframe extends Node implements RigidBodyWireframe
{
    /** The smallest float increment to deal with, for translation && rotations.*/
    private static final float SMALLEST_FLOAT_CHANGE = 0.001f;

    /** The mapping between each chlid shape of the compound and it's JME Spatial representation.*/
    private final Map<CompoundShapeChild, TriMeshWireframe> bulletObjects = new HashMap<CompoundShapeChild, TriMeshWireframe>();

    /**
     *  Creates a concave compound rigid body mesh from the given shape.
     *
     * @param compound the shape that contains primitive (convex) children.
     */
    public CompoundWireframe( CompoundShape compound )
    {
        // Get the convex children that comprise the compound shape
        List<CompoundShapeChild> childShapes = compound.getChildList();
        assert childShapes != null : "The list of children was null - cannot create a compound shape from nothng";
        assert childShapes.size() > 0 : "The list of children was empty - cannot create a compound shape from nothing";

        // Create the appropiate size array for the wireframes of the child shapes
        TriMeshWireframe wireframe;
        Vector3f bulletTranslation;
        Quaternion bulletRotation = new Quaternion();

        for ( CompoundShapeChild child : childShapes )
        {
            // Create the chid wireframe shape
            wireframe = createChildWireframe( child.childShape );

            // Initialise the translation / rotation           
            Matrix3f matrix = child.transform.basis;
            bulletRotation.fromRotationMatrix( matrix.m00, matrix.m01, matrix.m02,
                    matrix.m10, matrix.m11, matrix.m12,
                    matrix.m20, matrix.m21, matrix.m22 );
            bulletTranslation = child.transform.origin;

            wireframe.updateWorldTranslation( bulletTranslation );
            wireframe.updateWorldRotation( bulletRotation );
            wireframe.updateGeometricState( 0f, true );

            // Store in the child listing
            bulletObjects.put( child, wireframe );

            // Store for rendering
            attachChild( wireframe );
        }

        updateGeometricState( 0f, true );
        setModelBound( new BoundingBox() );
        updateModelBound();
    }

    @Override
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

    @Override
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

    @Override
    public void updateActivityState( WireframeActivityState bulletState )
    {
        Iterator<TriMeshWireframe> it = bulletObjects.values().iterator();

        while ( it.hasNext() )
        {
            it.next().updateActivityState( bulletState );
        }
    }

    @Override
    public void render( Renderer renderer )
    {
        renderer.draw( this );
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

    /**
     *  Creates the appropiate wireframe for the child of the compound shape.
     *
     * @param shape the collsion shape of the child wireframe to create.
     * @return the created child wireframe, never <code>null</code>.
     */
    private TriMeshWireframe createChildWireframe( CollisionShape shape )
    {
        TriMeshWireframe wireframe = null;

        // Assert the CollisionShape is of the appropiate type
        assert shape instanceof ConvexShape : "Expecting CollisionShape to be a ConvexShape";
        if (shape instanceof ConvexShape) {
	        // Create a wireframe for the Convex Shape
	        wireframe = new ConvexWireframe( (ConvexShape) shape );
        } else if (shape instanceof ConcaveShape) {
        	// Create a wireframe for the Concave Shape
        	wireframe = new ConcaveWireframe( (ConcaveShape) shape );
        }

        //TODO hack - either all child shapes are convex or they could be anything - find out which
        if ( wireframe == null )
        {
            throw new IllegalArgumentException( "Shape not yet catered for: " + shape.getName() );
        }

        return wireframe;
    }
}
