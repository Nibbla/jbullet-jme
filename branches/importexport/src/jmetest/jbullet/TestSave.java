package jmetest.jbullet;

import java.io.File;

import com.jme.app.SimpleGame;
import com.jme.input.MouseInput;
import com.jme.math.Vector3f;
import com.jme.scene.Node;
import com.jme.scene.shape.Box;
import com.jme.scene.shape.Capsule;
import com.jme.scene.shape.Cylinder;
import com.jme.scene.shape.Sphere;
import com.jme.util.export.binary.BinaryExporter;
import com.jme.util.export.binary.BinaryImporter;
import com.jmex.game.state.GameStateManager;
import com.jmex.jbullet.PhysicsSpace;
import com.jmex.jbullet.collision.shapes.CollisionShape;
import com.jmex.jbullet.joints.PhysicsHingeJoint;
import com.jmex.jbullet.joints.PhysicsPoint2PointJoint;
import com.jmex.jbullet.nodes.PhysicsNode;

public class TestSave extends SimpleGame {
	final PhysicsSpace pSpace=PhysicsSpace.getPhysicsSpace();
	@Override
	protected void simpleInitGame() {
		MouseInput.get().setCursorVisible(true);
		
		 // Add a physics sphere to the world
        Sphere sphere=new Sphere("physicssphere",16,16,1f);
        PhysicsNode physicsSphere=new PhysicsNode(sphere,CollisionShape.ShapeTypes.SPHERE);
        physicsSphere.setLocalTranslation(new Vector3f(3,6,0));
       rootNode.attachChild(physicsSphere);
        physicsSphere.updateRenderState();
        pSpace.add(physicsSphere);

      //   Add a physics sphere to the world using the collision shape from sphere one
        Sphere sphere2=new Sphere("physicssphere",16,16,1f);
        PhysicsNode physicsSphere2=new PhysicsNode(sphere2,physicsSphere.getCollisionShape());
        physicsSphere2.setLocalTranslation(new Vector3f(4,8,0));
        rootNode.attachChild(physicsSphere2);
        physicsSphere2.updateRenderState();
        pSpace.add(physicsSphere2);

        // Add a physics box to the world
        Box boxGeom=new Box("physicsbox",Vector3f.ZERO,1f,1f,1f);
        PhysicsNode physicsBox=new PhysicsNode(boxGeom,CollisionShape.ShapeTypes.BOX);
        physicsBox.setFriction(0.1f);
        physicsBox.setLocalTranslation(new Vector3f(.6f,4,.5f));
        rootNode.attachChild(physicsBox);
        physicsBox.updateRenderState();
        pSpace.add(physicsBox);

        Cylinder cylGeom=new Cylinder("physicscyliner",16,16,1f,3f);
        PhysicsNode physicsCylinder=new PhysicsNode(cylGeom, CollisionShape.ShapeTypes.CYLINDER);
        physicsCylinder.setLocalTranslation(new Vector3f(-5,4,0));
        rootNode.attachChild(physicsCylinder);
        physicsCylinder.updateRenderState();
        pSpace.add(physicsCylinder);

        Capsule capGeom=new Capsule("physicscapsule",16,16,16,0.5f,2f);
        PhysicsNode physicsCapsule=new PhysicsNode(capGeom, CollisionShape.ShapeTypes.CAPSULE);
        physicsCapsule.setFriction(0.1f);
        physicsCapsule.setLocalTranslation(new Vector3f(-8,4,0));
        rootNode.attachChild(physicsCapsule);
        physicsCapsule.updateRenderState();
        pSpace.add(physicsCapsule);
        physicsCapsule.setMass(10f);

        // Join the physics objects with a Point2Point joint
        PhysicsPoint2PointJoint joint=new PhysicsPoint2PointJoint(physicsSphere, physicsBox, new Vector3f(-2,0,0), new Vector3f(2,0,0));
     //   PhysicsHingeJoint joint=new PhysicsHingeJoint(physicsSphere, physicsBox, new Vector3f(-2,0,0), new Vector3f(2,0,0), Vector3f.UNIT_Z,Vector3f.UNIT_Z);
        pSpace.add(joint);
      //  joint.enableMotor(true, 1, 20);
        // an obstacle mesh, does not move (mass=0)
        PhysicsNode node2=new PhysicsNode(new Sphere("physicsobstaclemesh",16,16,1.2f),CollisionShape.ShapeTypes.MESH,0);
        node2.setLocalTranslation(new Vector3f(2.5f,-4,0f));
        rootNode.attachChild(node2);
        node2.updateRenderState();
        pSpace.add(node2);

        // the floor, does not move (mass=0)
        PhysicsNode node3=new PhysicsNode(new Box("physicsfloor",Vector3f.ZERO,100f,0.2f,100f),CollisionShape.ShapeTypes.BOX,0);
        node3.setLocalTranslation(new Vector3f(0f,-6,0f));
        rootNode.attachChild(node3);
        node3.updateRenderState();
        pSpace.add(node3);

        
       
        
        try {
            BinaryExporter.getInstance().save(rootNode, new File("/home/pau/prova.jme"));
           // XMLExporter.getInstance().save(state.getRootNode(), new File("/home/pau/prova.xml"));
        } catch (Exception e) {
          
        }
       	 
	          rootNode.updateRenderState();
	          rootNode.updateGeometricState(0, true);
	        pause=false;
	}
	
	@Override
	protected void simpleUpdate() {
		// TODO Auto-generated method stub
		super.simpleUpdate();
		pSpace.update(tpf);
	}
	public static void main(String[] args) throws Exception {
		TestSave t = new TestSave();
		t.start();
	}
}
