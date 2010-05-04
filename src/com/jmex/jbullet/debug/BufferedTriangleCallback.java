package com.jmex.jbullet.debug;

import com.bulletphysics.collision.shapes.TriangleCallback;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.FloatBuffer;
import java.util.ArrayList;
import javax.vecmath.Vector3f;

/**
 *
 * @author CJ Hare
 */
public class BufferedTriangleCallback implements TriangleCallback
{
    //TODO comment
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

    public void reset()
    {
        vertices.clear();
    }

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
