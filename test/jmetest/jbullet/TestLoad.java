package jmetest.jbullet;

import java.io.File;

import com.jme.app.SimpleGame;
import com.jme.input.MouseInput;
import com.jme.scene.Node;
import com.jme.util.export.binary.BinaryImporter;
import com.jmex.jbullet.PhysicsSpace;

public class TestLoad extends SimpleGame {
	final PhysicsSpace pSpace=PhysicsSpace.getPhysicsSpace();
	@Override
	protected void simpleInitGame() {
		MouseInput.get().setCursorVisible(true);
		// pSpace.setupBinaryClassLoader();
		 Node t=null;
	        try {
	        	 t = (Node) BinaryImporter.getInstance().load(new File("/home/pau/prova.jme"));
	            
	        } catch (Exception e) {
	          e.printStackTrace();
	        }
	        
	        //rootNode.attachChild(t);
       	 rootNode=t;
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
		TestLoad t = new TestLoad();
		t.start();
	}
}
