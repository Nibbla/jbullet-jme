package com.jmex.jbullet.debug;

import com.bulletphysics.collision.broadphase.BroadphaseNativeType;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.CompoundShape;
import com.bulletphysics.collision.shapes.ConcaveShape;
import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.collision.shapes.PolyhedralConvexShape;
import com.bulletphysics.collision.shapes.ShapeHull;
import com.bulletphysics.collision.shapes.TriangleCallback;
import com.bulletphysics.linearmath.Transform;
import com.bulletphysics.util.IntArrayList;
import com.bulletphysics.util.ObjectPool;
import java.util.List;
import javax.vecmath.Vector3f;

/**
 *  This class is very loosely based on GLShapeDrawer from jbullet (@author jezek2).
 *  <p/>
 *  The purpose is to retrieve the rigid body of the collision shape and draw it.
 *
 *  @author CJ Hare
 */
public class GLShapeDrawer
{
    /**
     * The transformMatrix that holds the scale, rotation and transform for a object to render - the object transform.
     */
    private static float[] transformMatrix = new float[16];

    /**
     *  Draws the wireframe for the given object.
     *
     * @param gl the renderer context.
     * @param objectTransform the matrix containing the objects transform (rotation, scale, translation).
     * @param shape the object whose physics bounds to render as a wireframe.
     * @param colour the colour to use when drawing the objects wireframe lines.
     */
    public static void drawWireframeObject( IGL gl, Transform objectTransform, CollisionShape shape, Vector3f colour )
    {
        // Assert the colours are in the expected range
        assert colour.x >= 0f && colour.x <= 1f : "OpenGL colour needs to be in range of 0 - 1 (inclusive) : " + colour.x;
        assert colour.y >= 0f && colour.y <= 1f : "OpenGL colour needs to be in range of 0 - 1 (inclusive) : " + colour.y;
        assert colour.z >= 0f && colour.z <= 1f : "OpenGL colour needs to be in range of 0 - 1 (inclusive) : " + colour.z;

        gl.glPushMatrix();
        objectTransform.getOpenGLMatrix( transformMatrix );
        gl.glMultMatrix( transformMatrix );

        if ( shape.getShapeType() == BroadphaseNativeType.COMPOUND_SHAPE_PROXYTYPE )
        {
            ObjectPool<Transform> transformsPool = ObjectPool.get( Transform.class );

            CompoundShape compoundShape = (CompoundShape) shape;
            Transform childTrans = transformsPool.get();
            for ( int i = compoundShape.getNumChildShapes() - 1; i >= 0; i-- )
            {
                compoundShape.getChildTransform( i, childTrans );
                CollisionShape colShape = compoundShape.getChildShape( i );
                drawWireframeObject( gl, childTrans, colShape, colour );
            }
            transformsPool.release( childTrans );
        }
        else
        {
            ObjectPool<Vector3f> vectorsPool = ObjectPool.get( Vector3f.class );

            gl.glDisable( IGL.GL_LIGHTING );
            gl.glEnable( IGL.GL_COLOR_MATERIAL );
            gl.glColor3f( colour.x, colour.y, colour.z );

            /*
             * Time to draw the wireframes!
             */
            if ( shape.isPolyhedral() )
            {
                // Assert the CollisionShape is of the appropiate type
                assert shape instanceof PolyhedralConvexShape : "Expecting CollisionShape to be a PolyhedralConvexShape";
                PolyhedralConvexShape polyhedralShape = (PolyhedralConvexShape) shape;

                // Render the wireframe of the shape
                drawWireframe( gl, polyhedralShape, vectorsPool );
            }
            else if ( shape.isConvex() )
            {
                // Assert the CollisionShape is of the appropiate type
                assert shape instanceof ConvexShape : "Expecting CollisionShape to be a ConvexShape";
                ConvexShape convexShape = (ConvexShape) shape;

                // Render the wireframe of the shape
                drawWireframe( gl, convexShape, vectorsPool );
            }
            else if ( shape.isConcave() )
            {
                // Assert the CollisionShape is of the appropiate type
                assert shape instanceof ConcaveShape : "Expecting CollisionShape to be a ConcaveShape";
                ConcaveShape concaveShape = (ConcaveShape) shape;

                // Render the wireframe of the shape
                drawWireframe( gl, concaveShape, vectorsPool );
            }

            gl.glEnable( IGL.GL_LIGHTING );
        }

        gl.glPopMatrix();
    }

    private static final void drawWireframe( IGL gl, PolyhedralConvexShape shape, ObjectPool<Vector3f> vectorsPool )
    {
        gl.glBegin( IGL.GL_LINES );

        Vector3f a = vectorsPool.get(), b = vectorsPool.get();
        for ( int i = 0; i < shape.getNumEdges(); i++ )
        {
            shape.getEdge( i, a, b );

            gl.glVertex3f( a.x, a.y, a.z );
            gl.glVertex3f( b.x, b.y, b.z );
        }
        gl.glEnd();

        vectorsPool.release( a );
        vectorsPool.release( b );
    }

    private static final void drawWireframe( IGL gl, ConvexShape shape, ObjectPool<Vector3f> vectorsPool )
    {
        // Check there is a hull shape to render
        if ( shape.getUserPointer() == null )
        {
            // create a hull approximation
            ShapeHull hull = new ShapeHull( shape );
            float margin = shape.getMargin();
            hull.buildHull( margin );
            shape.setUserPointer( hull );
        }

        // Assert state - should have a pointer to a hull (shape) that'll be drawn
        assert shape.getUserPointer() != null : "Should have a shape for the userPointer, instead got null";
        ShapeHull hull = (ShapeHull) shape.getUserPointer();

        if ( hull.numTriangles() > 0 )
        {
            // Loop variables
            final IntArrayList indicies = hull.getIndexPointer();
            final List<Vector3f> vertices = hull.getVertexPointer();
            int i1, i2, i3, index1, index2, index3, index = 0;
            Vector3f v1, v2, v3;

            gl.glBegin( IGL.GL_LINES );

            for ( int i = 0; i < hull.numTriangles(); i++ )
            {
                i1 = index++;
                i2 = index++;
                i3 = index++;

                // Assert the indicies are within bounds
                assert (i1 < hull.numIndices()
                        && i2 < hull.numIndices()
                        && i3 < hull.numIndices());

                index1 = indicies.get( i1 );
                index2 = indicies.get( i2 );
                index3 = indicies.get( i3 );

                // Assert the vertice numbers are within bounds
                assert (index1 < hull.numVertices()
                        && index2 < hull.numVertices()
                        && index3 < hull.numVertices());

                v1 = vertices.get( index1 );
                v2 = vertices.get( index2 );
                v3 = vertices.get( index3 );

                // Put the three lines that make up the triangle into the OpenGL render queue
                gl.glVertex3f( v1.x, v1.y, v1.z );
                gl.glVertex3f( v2.x, v2.y, v2.z );
                gl.glVertex3f( v3.x, v3.y, v3.z );
                gl.glVertex3f( v2.x, v2.y, v2.z );
                gl.glVertex3f( v3.x, v3.y, v3.z );
                gl.glVertex3f( v1.x, v1.y, v1.z );
            }
            gl.glEnd();
        }
    }

    private static final void drawWireframe( IGL gl, ConcaveShape shape, ObjectPool<Vector3f> vectorsPool )
    {
        //todo pass camera, for some culling
        Vector3f aabbMax = vectorsPool.get();
        aabbMax.set( 1e30f, 1e30f, 1e30f );
        Vector3f aabbMin = vectorsPool.get();
        aabbMin.set( -1e30f, -1e30f, -1e30f );

        GlWireframeDrawcallback drawCallback = new GlWireframeDrawcallback( gl );

        shape.processAllTriangles( drawCallback, aabbMin, aabbMax );

        vectorsPool.release( aabbMax );
        vectorsPool.release( aabbMin );
    }

    private static class GlWireframeDrawcallback implements TriangleCallback
    {
        private IGL gl;

        public GlWireframeDrawcallback( IGL gl )
        {
            this.gl = gl;
        }

        @Override
        public void processTriangle( Vector3f[] triangle, int partId, int triangleIndex )
        {
            gl.glBegin( IGL.GL_LINES );
            gl.glColor3f( 1, 0, 0 );
            gl.glVertex3f( triangle[0].x, triangle[0].y, triangle[0].z );
            gl.glVertex3f( triangle[1].x, triangle[1].y, triangle[1].z );
            gl.glColor3f( 0, 1, 0 );
            gl.glVertex3f( triangle[2].x, triangle[2].y, triangle[2].z );
            gl.glVertex3f( triangle[1].x, triangle[1].y, triangle[1].z );
            gl.glColor3f( 0, 0, 1 );
            gl.glVertex3f( triangle[2].x, triangle[2].y, triangle[2].z );
            gl.glVertex3f( triangle[0].x, triangle[0].y, triangle[0].z );
            gl.glEnd();
        }
    }
}
