package com.jmex.jbullet;

import com.g3d.font.BitmapFont;
import com.g3d.font.BitmapText;
import com.g3d.input.FlyByCamera;
import com.g3d.input.KeyInput;
import com.g3d.input.binding.BindingListener;
import com.g3d.material.RenderState;
import com.g3d.math.Quaternion;
import com.g3d.math.Vector3f;
import com.g3d.renderer.Camera;
import com.g3d.renderer.RenderManager;
import com.g3d.renderer.Renderer;
import com.g3d.renderer.queue.RenderQueue.Bucket;
import com.g3d.scene.Node;
import com.g3d.scene.Spatial.CullHint;
import com.g3d.system.AppSettings;
import com.g3d.system.G3DContext.Type;
import com.g3d.system.G3DSystem;
import com.g3d.util.BufferUtils;
import java.net.URL;

/**
 * <code>SimpleApplication</code> extends the <code>Application</code> class
 * to provide default functionality like a first-person camera,
 * and an accessible root node that is updated and rendered regularly.
 */
public abstract class SimplePhysicsApplication extends PhysicsApplication {

    protected Node rootNode = new Node("Root Node");
    protected Node guiNode = new Node("Gui Node");

    protected float secondCounter = 0.0f;
    protected BitmapText fpsText;

    protected FlyByCamera flyCam;
    protected boolean showSettings = true;

    public SimplePhysicsApplication(){
        super();
        
        // set some default settings in-case
        // settings dialog is not shown
        setSettings(new AppSettings(true));
    }

    @Override
    public void start(){
        // show settings dialog
        if (showSettings){
            if (!G3DSystem.showSettingsDialog(settings))
                return;
        }

        super.start();
    }

    public boolean isShowSettings() {
        return showSettings;
    }

    public void setShowSettings(boolean showSettings) {
        this.showSettings = showSettings;
    }

    public void loadFPSText(){
        BitmapFont font = manager.loadFont("cooper.fnt");

        fpsText = new BitmapText(font, false);
        fpsText.setSize(font.getCharSet().getRenderedSize());
        fpsText.setLocalTranslation(0, fpsText.getLineHeight(), 0);
        fpsText.setText("Frames per second");
        guiNode.attachChild(fpsText);
    }

    @Override
    public void initialize(){
        super.initialize();

        // enable depth test and back-face culling for performance
        renderer.applyRenderState(RenderState.DEFAULT);

        guiNode.setQueueBucket(Bucket.Gui);
        guiNode.setCullHint(CullHint.Never);
        loadFPSText();
        viewPort.attachScene(rootNode);
        guiViewPort.attachScene(guiNode);

        if (inputManager != null){
            flyCam = new FlyByCamera(cam);
            flyCam.setMoveSpeed(1f);
            flyCam.registerWithDispatcher(inputManager);

            if (context.getType() == Type.Display){
                inputManager.registerKeyBinding("SIMPLEAPP_Exit", KeyInput.KEY_ESCAPE);
            }

            inputManager.registerKeyBinding("SIMPLEAPP_CameraPos", KeyInput.KEY_C);
            inputManager.registerKeyBinding("SIMPLEAPP_Memory",    KeyInput.KEY_M);
            inputManager.addTriggerListener(new BindingListener() {
                public void onBinding(String binding, float value) {
                    if (binding.equals("SIMPLEAPP_Exit")){
                        stop();
                    }else if (binding.equals("SIMPLEAPP_CameraPos")){
                        if (cam != null){
                            Vector3f loc = cam.getLocation();
                            Quaternion rot = cam.getRotation();
                            System.out.println("Camera Position: ("+
                                    loc.x+", "+loc.y+", "+loc.z+")");
                            System.out.println("Camera Position: "+rot);
                        }
                    }else if (binding.equals("SIMPLEAPP_Memory")){
                        BufferUtils.printCurrentDirectMemory(null);
                    }
                }
            });
        }
        //start physics
        startPhysics();
        // call user code
        simpleInitApp();
    }

    @Override
    public void update() {
        if (speed == 0)
            return;
        
        super.update();
        float tpf = timer.getTimePerFrame();

        secondCounter += tpf;
        int fps = (int) timer.getFrameRate();
        if (secondCounter >= 1.0f){
            fpsText.setText("Frames per second: "+fps);
            secondCounter = 0.0f;
        }
        
        simpleUpdate(tpf);
        rootNode.updateLogicalState(tpf);
        guiNode.updateLogicalState(tpf);
        rootNode.updateGeometricState();
        guiNode.updateGeometricState();

        renderManager.render(tpf);
        simpleRender(renderManager);
    }

    @Override
    public void physicsUpdate(float tpf) {
        super.physicsUpdate(tpf);
        simplePhysicsUpdate(tpf);
    }

    public abstract void simpleInitApp();

    public void simpleUpdate(float tpf){
    }

    public void simplePhysicsUpdate(float tpf){
    }

    public void simpleRender(RenderManager rm){
    }

}
