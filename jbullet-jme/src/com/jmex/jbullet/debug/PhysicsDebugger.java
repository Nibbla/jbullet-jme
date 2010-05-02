package com.jmex.jbullet.debug;

import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.CollisionObjectType;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.ConcaveShape;
import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.collision.shapes.ShapeHull;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.IntArrayList;
import com.jme.bounding.BoundingBox;
import com.jme.light.DirectionalLight;
import com.jme.math.Quaternion;
import com.jme.renderer.ColorRGBA;
import com.jme.renderer.Renderer;
import com.jme.scene.Spatial;
import com.jme.scene.TriMesh;
import com.jme.scene.VBOInfo;
import com.jme.scene.state.LightState;
import com.jme.scene.state.MaterialState;
import com.jme.scene.state.WireframeState;
import com.jme.scene.state.ZBufferState;
import com.jme.system.DisplaySystem;
import com.jmex.jbullet.PhysicsSpace;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.nio.IntBuffer;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
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
    //TODO refactor out the wireframe stuff into a new class - extend TriMesh
    private enum StateColour
    {
        A( ColorRGBA.red ), B( ColorRGBA.blue );

        public final MaterialState colour;

        StateColour( ColorRGBA stateColour )
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
    /** The smallest float increment to deal with, for translation && rotations.*/
    private static final float SMALLEST_FLOAT_CHANGE = 0.001f;

    /** Temporary Transform, used to store the bullet object's transform.*/
    private static final Transform transform = new Transform();

    /** Temporary Quaternion, used to covert between transform.basis and JME WorldRotation. */
    private static final Quaternion worldRotation = new Quaternion();

    /** The standard ordering of objects by their z order, keeps things correctly in perspective.*/
    private static final ZBufferState zOrdered;

    /** The wireframe state shared that makes the renderer draw wireframes, not fill in the triangles.*/
    private static final WireframeState wireframeState;

    /** The single light state applied to all the physics bounds meshes, get the MaterialState to work (saves on geometry colours)*/
    private static final LightState lighting;

    /** The collision object assigned to a physics body mapped to a JME Spatial representation.*/
    private static final Map<CollisionObject, Spatial> bulletObjects = new HashMap<CollisionObject, Spatial>();

    // Static initialisation
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

    //TODO commnet
    public static void drawWireframes( Renderer renderer )
    {
        DynamicsWorld dynamicsWorld = PhysicsSpace.getPhysicsSpace().getDynamicsWorld();

        if ( dynamicsWorld != null )
        {
            CollisionShape shape;
            Vector3f worldTranslation;
            Spatial wireframe;

            for ( CollisionObject collisionObject : dynamicsWorld.getCollisionObjectArray() )
            {
                // Get the world translation of the object - determines translation, rotation and scale
                collisionObject.getWorldTransform( transform );

                shape = collisionObject.getCollisionShape();

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

                worldTranslation = transform.origin;

                // Does the object already have a Jme Spatial
                wireframe = bulletObjects.get( collisionObject );

                // If there's not a Spatial of the collision object - one needs creating!
                if ( wireframe == null )
                {
                    //TODO shapes can be polyhedral as well as being other types - the convex & concave shapes have access
                    // indicies and vertices
                    if ( shape.isConvex() )
                    {
                        // Assert the CollisionShape is of the appropiate type
                        assert shape instanceof ConvexShape : "Expecting CollisionShape to be a ConvexShape";
                        ConvexShape convexShape = (ConvexShape) shape;

                        // Create a wireframe for the Convex Shape
                        wireframe = createWireframe( convexShape );
                    }
                    else if ( shape.isConcave() )
                    {
                        // Assert the CollisionShape is of the appropiate type
                        assert shape instanceof ConcaveShape : "Expecting CollisionShape to be a ConcaveShape";
                        ConcaveShape concaveShape = (ConcaveShape) shape;

                        // Create a wireframe for the Concave Shape
                        wireframe = createWireframe( concaveShape );
                    }

                    //TODO null check is a hack
                    if ( wireframe != null )
                    {
                        // Translate the Spatial to the correct location in the world
                        wireframe.setLocalTranslation( worldTranslation.x, worldTranslation.y, worldTranslation.z );

                        // Set the local rotation, without the Spatial keeping a reference to my worldRotation
                        wireframe.getLocalRotation().set( worldRotation );

                        // The rotation change needs picking up (as the rotation was set directly)
                        wireframe.updateWorldVectors();
                    }

                    // Store the created wireframe, so we don't have to recreate it
                    bulletObjects.put( collisionObject, wireframe );

                    //TODO deal with infinite shape?
                    //TODO what happens to composite shapes?
                }

                // TODO this null check is a short term hack - compound & infinite shapes will be null
                if ( wireframe != null )
                {
                    // Check if the worldTranslation of the object has changed
                    if ( isNotEqual( worldTranslation, wireframe.getWorldTranslation() ) )
                    {
                        // The physics object has moved since last render - update the world translation of the Spatial
                        wireframe.setLocalTranslation( worldTranslation.x, worldTranslation.y, worldTranslation.z );
                        wireframe.updateWorldVectors();
                    }

                    // Check if the worldRotation of the object has changed
                    if ( isNotEqual( worldRotation, wireframe.getWorldRotation() ) )
                    {
                        // The physics object has rotated since last render - update the world rotation of the Spatial
                        wireframe.getLocalRotation().set( worldRotation );
                        wireframe.updateWorldVectors();
                    }

                    renderer.draw( wireframe );
                }
            }
        }
    }
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
    private static Spatial createWireframe( ConcaveShape concaveShape )
    {
        // Create the call back that'll create the vertex buffer
        BufferedTriangleCallback triangleProcessor = new BufferedTriangleCallback();
        concaveShape.processAllTriangles( triangleProcessor, aabbMin, aabbMax );

        // Retrieve the vextex and index buffers
        FloatBuffer vertices = triangleProcessor.getVertices();

        return createTriMeshWireframe( vertices );
    }

    /**
     *  Creates a wireframe from a Convex shape.
     *
     * @param convexShape the shape that the wireframe will be created from, define what shape the wireframe will take.
     * @return the wireframe Spatial that can be manipulated just like any other!
     */
    private static Spatial createWireframe( ConvexShape convexShape )
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

        return createTriMeshWireframe( vertices );
    }

    /**
     *  Creates a TriMesh setting it up so that it is rendered as a wireframe.
     *
     * @param vertices the vextex buffer that the index buffer refers to.
     * @return The create Spatial that is a wireframe with the shared states of all PhysicsDebugger wireframe objects.
     */
    private static Spatial createTriMeshWireframe( FloatBuffer vertices )
    {
        TriMesh mesh = new TriMesh();
        mesh.setVertexBuffer( vertices );
        mesh.setIndexBuffer( generateIndicies( vertices ) );

        // Create the model bounds for frustum culling
        mesh.setModelBound( new BoundingBox() );
        mesh.updateModelBound();

        // Apply the shared state for wireframe, drawing order and starting colour
        mesh.setRenderState( StateColour.A.colour );
        mesh.setRenderState( wireframeState );
        mesh.setRenderState( zOrdered );
        mesh.setRenderState( lighting );

        // The render state needs updating to pickup the change in colour
        mesh.updateRenderState();

        // Use Vertex Buffering to achieve substainally better rendering with high poly objects
        mesh.setVBOInfo( new VBOInfo( true ) );

        return mesh;
    }

    /**
     *  Generates a normally ordered index buffer for the given number of triangles.<br/>
     *  Basically when a array of vertex are ordered where each three vertex make a triangle generateIndicies()
     *  will generate the correct index to match (needed for TriMesh).
     *
     * @param vertices the vertex buffer, for which the index buffer is being created.
     * @return the normally ordered index buffer (0,1,2,3,4....etc)
     */
    private static IntBuffer generateIndicies( FloatBuffer vertices )
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
    private static final boolean isNotEqual( Quaternion a, Quaternion b )
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
    private static final boolean isNotEqual( Vector3f a, com.jme.math.Vector3f b )
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
    private static final boolean isNotEqual( float a, float b )
    {
        return a > b + SMALLEST_FLOAT_CHANGE || b > a + SMALLEST_FLOAT_CHANGE;
    }
}
