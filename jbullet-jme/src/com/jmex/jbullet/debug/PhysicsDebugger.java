package com.jmex.jbullet.debug;

import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.dispatch.CollisionObjectType;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.ConcaveShape;
import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.collision.shapes.PolyhedralConvexShape;
import com.bulletphysics.collision.shapes.ShapeHull;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.IntArrayList;
import com.bulletphysics.util.ObjectPool;
import com.jme.bounding.BoundingBox;
import com.jme.math.Quaternion;
import com.jme.renderer.Renderer;
import com.jme.scene.Line;
import com.jme.scene.Spatial;
import com.jme.scene.VBOInfo;
import com.jme.scene.state.ZBufferState;
import com.jme.system.DisplaySystem;
import com.jmex.jbullet.PhysicsSpace;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
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
    /** The smallest float increment to deal with, for translation && rotations.*/
    private static final float SMALLEST_FLOAT_CHANGE = 0.001f;

    /** Temporary Transform, used to store the bullet object's transform.*/
    private static final Transform transform = new Transform();

    /** Temporary Quaternion, used to covert between transform.basis and JME WorldRotation. */
    private static final Quaternion worldRotation = new Quaternion();

    /** The standard ordering of objects by their z order, keeps things correctly in perspective.*/
    private static final ZBufferState zOrdered;

    /** The collision object assigned to a physics body mapped to a JME Spatial representation.*/
    private static final Map<CollisionObject, Spatial> bulletObjects = new HashMap<CollisionObject, Spatial>();

    // Static initialisation
    static
    {
        zOrdered = DisplaySystem.getDisplaySystem().getRenderer().createZBufferState();
        zOrdered.setWritable( true );
        zOrdered.setEnabled( true );
        zOrdered.setFunction( ZBufferState.TestFunction.LessThanOrEqualTo );
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

                        wireframe = createWireframe( convexShape, worldTranslation, worldRotation );
                    }
                    else if ( shape.isConcave() )
                    {
                        // Assert the CollisionShape is of the appropiate type
                        assert shape instanceof ConcaveShape : "Expecting CollisionShape to be a ConcaveShape";
                        ConcaveShape concaveShape = (ConcaveShape) shape;

                        wireframe = createWireframe( concaveShape, worldTranslation, worldRotation );
                    }
                    else if ( shape.isPolyhedral() )
                    {
                        //TODO why bother with polyhedral,  as it's a child of ConvexShape

                        // Assert the CollisionShape is of the appropiate type
                        assert shape instanceof PolyhedralConvexShape : "Expecting CollisionShape to be a PolyhedralConvexShape";
                        PolyhedralConvexShape polyhedralShape = (PolyhedralConvexShape) shape;

                        wireframe = createWireframe( polyhedralShape, worldTranslation, worldRotation );
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

                    wireframe.draw( renderer );
                }
            }
        }
    }

    //TODO colour
    private static Line createWireframe(
            PolyhedralConvexShape polyhedralShape, Vector3f worldTranslation, Quaternion worldRotation )
    {
        ObjectPool<Vector3f> vectorsPool = ObjectPool.get( Vector3f.class );
        Vector3f a = vectorsPool.get(), b = vectorsPool.get();

        int numberOfEdges = polyhedralShape.getNumEdges();

        int size = 3 * numberOfEdges * Float.SIZE;

        FloatBuffer vertices = ByteBuffer.allocateDirect( size ).order( ByteOrder.nativeOrder() ).asFloatBuffer();

        int colourSize = 4 * numberOfEdges * Float.SIZE;

        FloatBuffer colour = ByteBuffer.allocateDirect( colourSize ).order( ByteOrder.nativeOrder() ).asFloatBuffer();

        for ( int i = 0; i < numberOfEdges; i++ )
        {
            polyhedralShape.getEdge( i, a, b );

            vertices.put( a.x );
            vertices.put( a.y );
            vertices.put( a.z );

            vertices.put( b.x );
            vertices.put( b.y );
            vertices.put( b.z );

            colour.put( 1f ).put( 0f ).put( 1f ).put( 1 );
            colour.put( 0f ).put( 1f ).put( 0f ).put( 1 );
        }
        Line line = createLine( vertices, colour, worldTranslation, worldRotation );

        vectorsPool.release( a );
        vectorsPool.release( b );

        return line;
    }

    private static Line createWireframe(
            ConcaveShape concaveShape, Vector3f worldTranslation, Quaternion worldRotation )
    {
        //TODO refactor?
        //TODO figure out the purpose of an extremely large bounding box, how is it used? can the values be hard coded?
        ObjectPool<Vector3f> vectorsPool = ObjectPool.get( Vector3f.class );
        Vector3f aabbMax = vectorsPool.get();
        aabbMax.set(
                1e30f, 1e30f, 1e30f );
        Vector3f aabbMin = vectorsPool.get();
        aabbMin.set(
                -1e30f, -1e30f, -1e30f );

        BufferedTriangleCallback triangleProcessor = new BufferedTriangleCallback();

        concaveShape.processAllTriangles( triangleProcessor, aabbMin, aabbMax );

        FloatBuffer vertices = triangleProcessor.getVertices();
        FloatBuffer colour = triangleProcessor.getColour();
        return createLine( vertices, colour, worldTranslation, worldRotation );
    }

    private static Line createWireframe(
            ConvexShape convexShape, Vector3f worldTranslation, Quaternion worldRotation )
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

        int size = 3 * 3 * numberOfTriangles * Float.SIZE;
        FloatBuffer vertices = ByteBuffer.allocateDirect( size ).order( ByteOrder.nativeOrder() ).asFloatBuffer();

        int colourSize = 4 * 3 * numberOfTriangles * Float.SIZE;
        FloatBuffer colour = ByteBuffer.allocateDirect( colourSize ).order( ByteOrder.nativeOrder() ).asFloatBuffer();

        // Loop variables
        final IntArrayList hullIndicies = hull.getIndexPointer();
        final List<Vector3f> hullVertices = hull.getVertexPointer();
        Vector3f vertexA, vertexB, vertexC;
        int indexA, indexB, indexC;
        int index = 0;

        for ( int i = 0; i
                < numberOfTriangles; i++ )
        {
            indexA = index++;
            indexB = index++;
            indexC = index++;

            // Grab the data for this triangle from the hull
            vertexA = hullVertices.get( hullIndicies.get( indexA ) );
            vertexB = hullVertices.get( hullIndicies.get( indexB ) );
            vertexC = hullVertices.get( hullIndicies.get( indexC ) );

            vertices.put( vertexA.x ).put( vertexA.y ).put( vertexA.z );
            vertices.put( vertexB.x ).put( vertexB.y ).put( vertexB.z );

            vertices.put( vertexB.x ).put( vertexB.y ).put( vertexB.z );
            vertices.put( vertexC.x ).put( vertexC.y ).put( vertexC.z );

            vertices.put( vertexC.x ).put( vertexC.y ).put( vertexC.z );
            vertices.put( vertexA.x ).put( vertexA.y ).put( vertexA.z );

            for ( int six = 0; six
                    < 6; six++ )
            {
                colour.put( 1f ).put( 0f ).put( 1f ).put( 1f );
            }
        }

        return createLine( vertices, colour, worldTranslation, worldRotation );
    }

    private static Line createLine( FloatBuffer vertices, FloatBuffer colour, Vector3f worldTranslation, Quaternion worldRotation )
    {
        Line line = new Line();
        line.setVertexBuffer( vertices );
        line.setColorBuffer( colour );
        line.setLineWidth( 2f );
        line.generateIndices();

        line.setModelBound( new BoundingBox() );
        line.updateModelBound();

        line.updateRenderState();

        // Translate the Spatial to the correct location in the world
        line.setLocalTranslation( worldTranslation.x, worldTranslation.y, worldTranslation.z );

        // Set the local rotation, without the Spatial keeping a reference to my worldRotation
        line.getLocalRotation().set( worldRotation );

        line.setRenderState( zOrdered );

        // The rotation change needs picking up (as the rotation was set directly)
        line.updateWorldVectors();

        // The render state needs updating to pickup the change in colour
        line.updateRenderState();

        // Use Vertex Buffering to achieve substainally better rendering with high poly objects
        VBOInfo vbo = new VBOInfo( true );
        line.setVBOInfo( vbo );

        return line;
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
