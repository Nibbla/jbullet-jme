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

import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.CollisionObjectType;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.CompoundShape;
import com.bulletphysics.collision.shapes.ConcaveShape;
import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.linearmath.Transform;
import com.jme.math.Quaternion;
import com.jme.renderer.Renderer;
import com.jme.scene.Spatial;
import com.jme.util.geom.Debugger;
import com.jmex.jbullet.PhysicsSpace;
import java.util.HashMap;
import java.util.Map;
import java.util.logging.Logger;
import javax.vecmath.Matrix3f;
import javax.vecmath.Vector3f;

/**
 *  Deals with the rendering of the JME objects representation in Bullet.
 *  <p/>
 *  Note: currently only renders rigid bodies.
 *
 *  @author CJ Hare
 */
public class PhysicsDebugger
{
    /** The happy little logger*/
    private static final Logger LOG = Logger.getLogger( PhysicsDebugger.class.getName() );

    /** Temporary Transform, used to store the bullet object's transform.*/
    private static final Transform transform = new Transform();

    /** Temporary Quaternion, used to covert between transform.basis and JME WorldRotation. */
    private static final Quaternion worldRotation = new Quaternion();

    /** The collision object assigned to a physics body mapped to a JME Spatial representation.*/
    private static final Map<CollisionObject, RigidBodyWireframe> bulletObjects = new HashMap<CollisionObject, RigidBodyWireframe>();

    /**
     *  Renders all the current physics bounds for objects in the current dynamic world.
     *
     * @param renderer where the bounds will be rendered.
     */
    public static void drawWireframes( Renderer renderer )
    {
        DynamicsWorld dynamicsWorld = PhysicsSpace.getPhysicsSpace().getDynamicsWorld();

        if ( dynamicsWorld != null )
        {
            Vector3f worldTranslation;
            RigidBodyWireframe wireframe;

            for ( CollisionObject collisionObject : dynamicsWorld.getCollisionObjectArray() )
            {
                // Get the world translation of the object - determines translation, rotation and scale
                collisionObject.getWorldTransform( transform );

                retrieveAndStoreWorldRotation( collisionObject );
                worldTranslation = transform.origin;

                // Does the object already have a Jme Spatial
                wireframe = bulletObjects.get( collisionObject );

                // If there's not a Spatial of the collision object - one needs creating! (but only try it once)
                if ( wireframe == null && bulletObjects.keySet().contains( collisionObject ) == false )
                {
                    wireframe = createWireframe( collisionObject );

                    // If the returned wireframe is null, then the shape is not yet supported
                    if ( wireframe != null )
                    {
                        wireframe.updateWorldTranslation( worldTranslation );
                        wireframe.updateWorldRotation( worldRotation );
                    }

                    // Store the created wireframe, so we don't have to recreate it (or try again if it's not supported)
                    bulletObjects.put( collisionObject, wireframe );
                }

                // If the wireframe is null @here then the shape type is not yet supported
                if ( wireframe != null )
                {
                    wireframe.updateWorldTranslation( worldTranslation );
                    wireframe.updateWorldRotation( worldRotation );
                    wireframe.render( renderer );
                }
            }
        }
    }

    /**
     *  Creates the appropiate wireframe to represent the bullet collision object.
     *  <p/>
     *  NOTE: when the collision object type is not supported a log entry is made.
     *
     * @param collisionObject the CollisionObject whose wireframe representation will be created.
     * @return the wireframe that jme can renderer, or <code>null</code> if the shape type is not currently supported.
     */
    private static RigidBodyWireframe createWireframe( CollisionObject collisionObject )
    {
        RigidBodyWireframe wireframe = null;

        //TODO covert to a switch?
        //TODO check for compond child movement

        CollisionShape shape = collisionObject.getCollisionShape();

//TODO         shape.isInfinite();

        if ( shape.isCompound() )
        {
            // Assert the CollisionShape is of the appropiate type
            assert shape instanceof CompoundShape : "Expecting CollisionShape to be a ConvexShape";
            CompoundShape compoundShape = (CompoundShape) shape;

            // Create a wireframe for the Compound Shape
            wireframe = new CompoundWireframe( compoundShape );
        }
        else if ( shape.isConvex() )
        {
            // Assert the CollisionShape is of the appropiate type
            assert shape instanceof ConvexShape : "Expecting CollisionShape to be a ConvexShape";
            ConvexShape convexShape = (ConvexShape) shape;

            // Create a wireframe for the Convex Shape
            wireframe = new ConvexWireframe( convexShape );
        }
        else if ( shape.isConcave() )
        {
            // Assert the CollisionShape is of the appropiate type
            assert shape instanceof ConcaveShape : "Expecting CollisionShape to be a ConcaveShape";
            ConcaveShape concaveShape = (ConcaveShape) shape;

            // Create a wireframe for the Concave Shape
            wireframe = new ConcaveWireframe( concaveShape );
        }

        // Did we have a problem trying to create the wireframe?
        if ( wireframe == null )
        {
            LOG.warning( "Could not create wireframe for: " + shape );
        }

        return wireframe;
    }

    /**
     *  World Rotation has a special case for GhostObjects as their matrix4f does not contain the scaling or rotation elements,
     *  which really they should.
     *
     * @param collisionObject the object to retrieve the world rotation from, or use default in the special case.
     */
    private static final void retrieveAndStoreWorldRotation( CollisionObject collisionObject )
    {
        // Ghosts get special measures, as their rotation & scale matrix is not initialised (blank)
        if ( isGhostObject( collisionObject ) )
        {
            // Just use the identity rotation (no rotation)
            worldRotation.identity();
        }
        else
        {
            Matrix3f matrix = transform.basis;
            worldRotation.fromRotationMatrix( matrix.m00, matrix.m01, matrix.m02,
                    matrix.m10, matrix.m11, matrix.m12,
                    matrix.m20, matrix.m21, matrix.m22 );
        }
    }

    /**
     *  Determines whether the given collision object is a GhostObject.<p/>
     *  Special treatment are needed for these as their transform (rotatiion / scale matrix)
     *  is not initialised.
     *
     * @param collisionObject the collision object to test the internal type for.
     * @return <code>true</code> the given collision object is a ghost, <code>false</code> otherwise.
     */
    private static final boolean isGhostObject( CollisionObject collisionObject )
    {
        return collisionObject.getInternalType() == CollisionObjectType.GHOST_OBJECT;
    }
}
