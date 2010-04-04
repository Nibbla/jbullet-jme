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
import com.jme.scene.state.ZBufferState;
import com.jme.system.DisplaySystem;
import com.jmex.jbullet.PhysicsSpace;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.List;
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
    //TODO commnet
    private static final Transform transform = new Transform();

    private static final Quaternion worldRotation = new Quaternion();

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

    public static void drawWireframes( Renderer renderer )
    {
        DynamicsWorld dynamicsWorld = PhysicsSpace.getPhysicsSpace().getDynamicsWorld();

        if ( dynamicsWorld != null )
        {
            CollisionShape shape;
            Vector3f worldTranslation;

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

                //TODO shapes can be polyhedral as well as being other types - the convex & concave shapes have access
                // indicies and vertices
                if ( shape.isConvex() )
                {
                    // Assert the CollisionShape is of the appropiate type
                    assert shape instanceof ConvexShape : "Expecting CollisionShape to be a ConvexShape";
                    ConvexShape convexShape = (ConvexShape) shape;

                    drawWireframe( renderer, convexShape, worldTranslation, worldRotation );
                }
                else if ( shape.isConcave() )
                {
                    // Assert the CollisionShape is of the appropiate type
                    assert shape instanceof ConcaveShape : "Expecting CollisionShape to be a ConcaveShape";
                    ConcaveShape concaveShape = (ConcaveShape) shape;

                    drawWireframe( renderer, concaveShape, worldTranslation, worldRotation );
                }
                else if ( shape.isPolyhedral() )
                {
                    //TODO why bother with polyhedral,  as it's a child of ConvexShape

                    // Assert the CollisionShape is of the appropiate type
                    assert shape instanceof PolyhedralConvexShape : "Expecting CollisionShape to be a PolyhedralConvexShape";
                    PolyhedralConvexShape polyhedralShape = (PolyhedralConvexShape) shape;

                    drawWireframe( renderer, polyhedralShape, worldTranslation, worldRotation );
                }
                //TODO deal with infinite shape?
                //TODO what happens to composite shapes?
            }
        }

    }

    //TODO colour
    public static final void drawWireframe(
            Renderer renderer, PolyhedralConvexShape polyhedralShape, Vector3f worldTranslation, Quaternion worldRotation )
    {
        ObjectPool<Vector3f> vectorsPool = ObjectPool.get( Vector3f.class );
        Vector3f a = vectorsPool.get(), b = vectorsPool.get();

        int numberOfEdges = polyhedralShape.getNumEdges();

        int size = 3 * numberOfEdges * Float.SIZE;

        FloatBuffer vertices = ByteBuffer.allocateDirect( size ).order( ByteOrder.nativeOrder() ).asFloatBuffer();

        int colourSize = 4 * numberOfEdges * Float.SIZE;

        FloatBuffer colour = ByteBuffer.allocateDirect( colourSize ).order( ByteOrder.nativeOrder() ).asFloatBuffer();

        for ( int i = 0;
                i < numberOfEdges;
                i++ )
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

        // Create & Init z-buffer state
        ZBufferState zBufferState = DisplaySystem.getDisplaySystem().getRenderer().createZBufferState();
        zBufferState.setWritable(
                true );
        zBufferState.setEnabled(
                true );
        zBufferState.setFunction( ZBufferState.TestFunction.LessThanOrEqualTo );

        line.setRenderState( zBufferState );

        // The rotation change needs picking up (as the rotation was set directly)
        line.updateWorldVectors();

        // The render state needs updating to pickup the change in colour
        line.updateRenderState();


        //TODO check whether this does frustum culling?
        //TODO need only to create objects once
        line.draw( renderer );

        vectorsPool.release( a );
        vectorsPool.release( b );
    }

    public static final void drawWireframe(
            Renderer renderer, ConcaveShape concaveShape, Vector3f worldTranslation, Quaternion worldRotation )
    {
        //TODO refactor
        ObjectPool<Vector3f> vectorsPool = ObjectPool.get( Vector3f.class );
        Vector3f aabbMax = vectorsPool.get();
        aabbMax.set(
                1e30f, 1e30f, 1e30f );
        Vector3f aabbMin = vectorsPool.get();
        aabbMin.set(
                -1e30f, -1e30f, -1e30f );

        BufferedTriangleCallback triangleProcessor = new BufferedTriangleCallback();

        concaveShape.processAllTriangles( triangleProcessor, aabbMin, aabbMax );

        Line line = new Line();
        line.setVertexBuffer( triangleProcessor.getVertices() );
        line.setColorBuffer( triangleProcessor.getColour() );
        line.setLineWidth(
                2f );
        line.generateIndices();

        line.setModelBound(
                new BoundingBox() );
        line.updateModelBound();

        line.updateRenderState();

        // Translate the Spatial to the correct location in the world
        line.setLocalTranslation( worldTranslation.x, worldTranslation.y, worldTranslation.z );

        // Set the local rotation, without the Spatial keeping a reference to my worldRotation
        line.getLocalRotation().set( worldRotation );

        // Create & Init z-buffer state
        ZBufferState zBufferState = DisplaySystem.getDisplaySystem().getRenderer().createZBufferState();
        zBufferState.setWritable(
                true );
        zBufferState.setEnabled(
                true );
        zBufferState.setFunction( ZBufferState.TestFunction.LessThanOrEqualTo );

        line.setRenderState( zBufferState );

        // The rotation change needs picking up (as the rotation was set directly)
        line.updateWorldVectors();

        // The render state needs updating to pickup the change in colour
        line.updateRenderState();


        //TODO check whether this does frustum culling?
        //TODO need only to create objects once
        line.draw( renderer );
    }

    public static final void drawWireframe(
            Renderer renderer, ConvexShape convexShape, Vector3f worldTranslation, Quaternion worldRotation )
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

        // Create & Init z-buffer state
        ZBufferState zBufferState = DisplaySystem.getDisplaySystem().getRenderer().createZBufferState();
        zBufferState.setWritable( true );
        zBufferState.setEnabled( true );
        zBufferState.setFunction( ZBufferState.TestFunction.LessThanOrEqualTo );

        line.setRenderState( zBufferState );

        // The rotation change needs picking up (as the rotation was set directly)
        line.updateWorldVectors();

        // The render state needs updating to pickup the change in colour
        line.updateRenderState();

        //TODO check whether this does frustum culling?
        //TODO need only to create objects once
        line.draw( renderer );
    }
}
