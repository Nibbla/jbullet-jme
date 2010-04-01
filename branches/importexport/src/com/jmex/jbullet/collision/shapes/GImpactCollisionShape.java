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
package com.jmex.jbullet.collision.shapes;

import com.bulletphysics.collision.shapes.BvhTriangleMeshShape;
import com.bulletphysics.extras.gimpact.GImpactMeshShape;
import com.jme.math.Vector3f;
import com.jme.scene.Node;
import com.jme.scene.Spatial;
import com.jme.scene.TriMesh;
import com.jme.util.export.InputCapsule;
import com.jme.util.export.JMEExporter;
import com.jme.util.export.JMEImporter;
import com.jme.util.export.OutputCapsule;
import com.jmex.jbullet.collision.shapes.CollisionShape.ShapeTypes;
import com.jmex.jbullet.util.Converter;

import java.io.IOException;
import java.util.List;

/**
 * Basic mesh collision shape
 * @author normenhansen
 */
public class GImpactCollisionShape extends CollisionShape{

	private TriMesh mesh;
	
    /**
     * creates a collision shape from the TriMesh leaf in the given node
     * @param node the node to get the TriMesh from
     */
    public GImpactCollisionShape(Node node) {
        createCollisionMesh(node);
    }
    
    public GImpactCollisionShape() {
        
    }

    /**
     * creates a collision shape from the given TriMesh
     * @param mesh the TriMesh to use
     */
    public GImpactCollisionShape(TriMesh mesh) {
        createCollisionMesh(mesh);
    }


    /**
     * creates a mesh that represents this node in the physics space. Can only be
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
        cShape=new GImpactMeshShape(Converter.convert(mesh));
        cShape.setLocalScaling(Converter.convert(mesh.getWorldScale()));
        ((GImpactMeshShape)cShape).updateBound();
        ((GImpactMeshShape)cShape).lockChildShapes();
        type=ShapeTypes.GIMPACT;
    }

	@Override
	public Class getClassTag() {
		// TODO Auto-generated method stub
		return null;
	}

	@Override
	public void read(JMEImporter im) throws IOException {
		InputCapsule capsule = im.getCapsule(this);
		mesh = (TriMesh) capsule.readSavable("mesh", null);
		cShape=new GImpactMeshShape(Converter.convert(mesh));
		Vector3f scale = (Vector3f) capsule.readSavable("scale", new Vector3f(1,1,1));
        cShape.setLocalScaling(Converter.convert(scale));
        type=ShapeTypes.GIMPACT;
	}

	@Override
	public void write(JMEExporter ex) throws IOException {
		OutputCapsule capsule = ex.getCapsule( this );
		GImpactMeshShape shape = (GImpactMeshShape)cShape;
		capsule.write(mesh, "mesh", null);
		javax.vecmath.Vector3f scale = new javax.vecmath.Vector3f();
		shape.getLocalScaling(scale);
		capsule.write(Converter.convert(scale), "scale", new Vector3f(1,1,1));
	}

}
