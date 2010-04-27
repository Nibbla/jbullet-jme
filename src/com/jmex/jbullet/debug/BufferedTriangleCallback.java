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
    private ArrayList<Vector3f> vertices;

    public BufferedTriangleCallback()
    {
        vertices = new ArrayList<Vector3f>();
    }

//    @Override
    public void processTriangle( Vector3f[] triangle, int partId, int triangleIndex )
    {
        // Three sets of individual lines
        // The new Vector is needed as the given triangle reference is from a pool
        vertices.add( new Vector3f( triangle[0] ) );
        vertices.add( new Vector3f( triangle[1] ) );
        vertices.add( new Vector3f( triangle[1] ) );
        vertices.add( new Vector3f( triangle[2] ) );
        vertices.add( new Vector3f( triangle[2] ) );
        vertices.add( new Vector3f( triangle[0] ) );
    }

    public void reset()
    {
        vertices.clear();
    }

    public FloatBuffer getVertices()
    {
        // There are 3 floats needed for each vertex (x,y,z)
        int bufferSize = vertices.size() * 3 * Float.SIZE;
        FloatBuffer verticesBuffer = ByteBuffer.allocateDirect( bufferSize ).order( ByteOrder.nativeOrder() ).asFloatBuffer();

        // Copy the values from the list to the direct float buffer
        for ( Vector3f v : vertices )
        {
            verticesBuffer.put( v.x ).put( v.y ).put( v.z );
        }

        return verticesBuffer;
    }

    public FloatBuffer getColour()
    {
        // There are four floats needed for each vertex (r.g.b.a)
        int bufferSize = vertices.size() * 4 * Float.SIZE;
        FloatBuffer colour = ByteBuffer.allocateDirect( bufferSize ).order( ByteOrder.nativeOrder() ).asFloatBuffer();

        // Create a buffer full of the colour blue
        for ( int i = 0; i < vertices.size(); i++ )
        {
            colour.put( 0f ).put( 0f ).put( 1f ).put( 1f );
        }

        return colour;

    }
}
