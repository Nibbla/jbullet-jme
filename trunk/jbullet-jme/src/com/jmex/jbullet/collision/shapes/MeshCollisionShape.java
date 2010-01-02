/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.jmex.jbullet.collision.shapes;

import com.bulletphysics.collision.shapes.BvhTriangleMeshShape;
import com.jme.scene.Node;
import com.jme.scene.Spatial;
import com.jme.scene.TriMesh;
import com.jmex.jbullet.collision.shapes.CollisionShape.Shapes;
import com.jmex.jbullet.util.Converter;
import java.util.List;

/**
 *
 * @author normenhansen
 */
public class MeshCollisionShape extends CollisionShape{

    public MeshCollisionShape(Node node) {
        createCollisionMesh(node);
    }

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
        cShape=new BvhTriangleMeshShape(Converter.convert(mesh),true);
        type=Shapes.MESH;
    }

}
