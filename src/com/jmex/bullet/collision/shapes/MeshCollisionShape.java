/*
 * Copyright (c) 2009-2010 Normen Hansen
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
package com.jmex.bullet.collision.shapes;

//import com.bulletphysics.collision.shapes.BvhTriangleMeshShape;
import com.jme.math.Vector3f;
import com.jme.scene.Node;
import com.jme.scene.Spatial;
import com.jme.scene.TriMesh;
import com.jmex.bullet.collision.shapes.CollisionShape.ShapeTypes;
//import com.jmex.jbullet.util.Converter;
import java.util.List;

/**
 * Basic mesh collision shape
 * @author normenhansen
 */
public class MeshCollisionShape extends CollisionShape{
    private TriMesh triMesh;

    /**
     * Creates a collision shape from the TriMesh leaf in the given node
     * @param node the node to get the TriMesh from
     */
    public MeshCollisionShape(Node node) {
        createCollisionMesh(node);
    }

    /**
     * Creates a collision shape from the given TriMesh
     * @param mesh the TriMesh to use
     */
    public MeshCollisionShape(TriMesh mesh) {
        createCollisionMesh(mesh);
    }


    /**
     * Creates a mesh that represents this node in the physics space. Can only be
     * used if this Node has one (and only one) TriMesh as a child.<br>
     */
    private void createCollisionMesh(Node node){
        List<Spatial> children=node.getChildren();
        if(children.size()==0){
            throw (new UnsupportedOperationException("PhysicsNode has no children, cannot compute collision mesh"));
        }
        else if(children.size()>1){
            throw (new UnsupportedOperationException("Can only create mesh from one single trimesh as leaf in this node."));
        }
        if(node.getChild(0) instanceof TriMesh){
            TriMesh mesh=(TriMesh)node.getChild(0);
            createCollisionMesh(mesh);
        }
        else{
            throw (new UnsupportedOperationException("No usable trimesh attached to this node!"));
        }
    }

    private void createCollisionMesh(TriMesh mesh){
        this.triMesh=mesh;
        cShape=createCollisionMesh();
        type=ShapeTypes.MESH;
    }

    protected float[] getVerticesArray() {
        //This is probably the LEAST efficient way to do this, but it'll work!
        Vector3f[] triangleVertices = new Vector3f[triMesh.getTriangleCount()*3];
        triMesh.getMeshAsTrianglesVertices(triangleVertices);
        float[] vertices=new float[triangleVertices.length * 3];
//        int[] triangles=new int[triangleVertices.length];
        Vector3f temp = new Vector3f();
        for(int loop=0; loop<triangleVertices.length; loop++)
        {
            triangleVertices[loop].mult(triMesh.getLocalScale(),temp);
            vertices[loop*3]=temp.x;
            vertices[loop*3+1]=temp.y;
            vertices[loop*3+2]=temp.z;
//            triangles[loop]=loop;
        }
        return vertices;
    }

    protected int[] getTrianglesArray() {
        //This is probably the LEAST efficient way to do this, but it'll work!
        Vector3f[] triangleVertices = new Vector3f[triMesh.getTriangleCount()*3];
        triMesh.getMeshAsTrianglesVertices(triangleVertices);
//        float[] vertices=new float[triangleVertices.length * 3];
        int[] triangles=new int[triangleVertices.length];
        Vector3f temp = new Vector3f();
        for(int loop=0; loop<triangleVertices.length; loop++)
        {
            triangleVertices[loop].mult(triMesh.getLocalScale(),temp);
//            vertices[loop*3]=temp.x;
//            vertices[loop*3+1]=temp.y;
//            vertices[loop*3+2]=temp.z;
            triangles[loop]=loop;
        }
        return triangles;
    }

    native long createCollisionMesh();

}
