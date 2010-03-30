package com.jmex.jbullet.debug;

import com.bulletphysics.collision.shapes.ConcaveShape;
import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.collision.shapes.PolyhedralConvexShape;
import com.bulletphysics.collision.shapes.ShapeHull;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.IntArrayList;
import com.bulletphysics.util.ObjectPool;
import com.jme.bounding.BoundingBox;
import com.jme.renderer.Renderer;
import com.jme.scene.Line;
import com.jme.scene.state.ZBufferState;
import com.jme.system.DisplaySystem;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.List;
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
    //TODO colour
    public static final void drawWireframe( Renderer renderer, PolyhedralConvexShape polyhedralShape, Transform transform )
    {
        Vector3f worldTranslation = transform.origin;

        //TODO world rotation
        //TODO world scale

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

        Line line = new Line();
        line.setVertexBuffer( vertices );

        line.setColorBuffer( colour );
        line.setLineWidth( 2f );
        line.generateIndices();

        line.setModelBound( new BoundingBox() );
        line.updateModelBound();

        line.updateRenderState();

        line.setLocalTranslation( worldTranslation.x, worldTranslation.y, worldTranslation.z );

        // Create & Init z-buffer state
        ZBufferState zBufferState = DisplaySystem.getDisplaySystem().getRenderer().createZBufferState();
        zBufferState.setWritable( true );
        zBufferState.setEnabled( true );
        zBufferState.setFunction( ZBufferState.TestFunction.LessThanOrEqualTo );

        line.setRenderState( zBufferState );
        line.updateRenderState();

        //TODO check whether this does frustum culling?
        //TODO need only to create objects once
        line.draw( renderer );

        vectorsPool.release( a );
        vectorsPool.release( b );
    }

    public static final void drawWireframe( Renderer renderer, ConcaveShape concaveShape, Transform transform )
    {
        Vector3f worldTranslation = transform.origin;

        //TODO refactor
        ObjectPool<Vector3f> vectorsPool = ObjectPool.get( Vector3f.class );
        Vector3f aabbMax = vectorsPool.get();
        aabbMax.set( 1e30f, 1e30f, 1e30f );
        Vector3f aabbMin = vectorsPool.get();
        aabbMin.set( -1e30f, -1e30f, -1e30f );

        BufferedTriangleCallback triangleProcessor = new BufferedTriangleCallback();

        concaveShape.processAllTriangles( triangleProcessor, aabbMin, aabbMax );

        Line line = new Line();
        line.setVertexBuffer( triangleProcessor.getVertices() );
        line.setColorBuffer( triangleProcessor.getColour() );
        line.setLineWidth( 2f );
        line.generateIndices();

        line.setModelBound( new BoundingBox() );
        line.updateModelBound();

        line.updateRenderState();

        line.setLocalTranslation( worldTranslation.x, worldTranslation.y, worldTranslation.z );

        // Create & Init z-buffer state
        ZBufferState zBufferState = DisplaySystem.getDisplaySystem().getRenderer().createZBufferState();
        zBufferState.setWritable( true );
        zBufferState.setEnabled( true );
        zBufferState.setFunction( ZBufferState.TestFunction.LessThanOrEqualTo );

        line.setRenderState( zBufferState );
        line.updateRenderState();

        //TODO check whether this does frustum culling?
        //TODO need only to create objects once
        line.draw( renderer );
    }

    public static final void drawWireframe( Renderer renderer, ConvexShape convexShape, Transform transform )
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

        Vector3f worldTranslation = transform.origin;

        //TODO world rotation
        //TODO world scale

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

        for ( int i = 0; i < numberOfTriangles; i++ )
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

            for ( int six = 0; six < 6; six++ )
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

        line.setLocalTranslation( worldTranslation.x, worldTranslation.y, worldTranslation.z );

        // Create & Init z-buffer state
        ZBufferState zBufferState = DisplaySystem.getDisplaySystem().getRenderer().createZBufferState();
        zBufferState.setWritable( true );
        zBufferState.setEnabled( true );
        zBufferState.setFunction( ZBufferState.TestFunction.LessThanOrEqualTo );

        line.setRenderState( zBufferState );
        line.updateRenderState();

        //TODO check whether this does frustum culling?
        //TODO need only to create objects once
        line.draw( renderer );
    }
}
