package com.jmex.jbullet;

import com.g3d.app.AppTask;
import com.g3d.input.JoyInput;
import com.g3d.input.KeyInput;
import com.g3d.input.MouseInput;
import com.g3d.system.*;
import com.g3d.math.Vector3f;
import com.g3d.renderer.Camera;
import com.g3d.renderer.Renderer;
import com.g3d.asset.AssetManager;
import com.g3d.input.InputManager;
import com.g3d.renderer.RenderManager;
import com.g3d.renderer.ViewPort;
import java.util.concurrent.Callable;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.Future;
import java.util.logging.Logger;

/**
 * The <code>Application</code> class represents an instance of a
 * real-time 3D rendering application.
 */
public class PhysicsApplication implements SystemListener {

    private static final Logger logger = Logger.getLogger(PhysicsApplication.class.getName());

    /**
     * The content manager. Typically initialized outside the GL thread
     * to allow offline loading of content.
     */
    protected AssetManager manager;

    protected Renderer renderer;
    protected RenderManager renderManager;
    protected ViewPort viewPort;
    protected ViewPort guiViewPort;

    protected G3DContext context;
    protected AppSettings settings;
    protected Timer timer;
    protected Camera cam;

    protected boolean inputEnabled = true;
    protected boolean pauseOnFocus = true;
    protected float speed = 1f;
    protected MouseInput mouseInput;
    protected KeyInput keyInput;
    protected JoyInput joyInput;
    protected InputManager inputManager;

    private final ConcurrentLinkedQueue<AppTask<?>> taskQueue = new ConcurrentLinkedQueue<AppTask<?>>();

    /**
     * Create a new instance of <code>Application</code>.
     */
    public PhysicsApplication(){
        // Why initialize it here? 
        // Because it allows offline loading of content.
        initContentManager();
    }

    public boolean isPauseOnLostFocus() {
        return pauseOnFocus;
    }

    public void setPauseOnLostFocus(boolean pauseOnLostFocus) {
        this.pauseOnFocus = pauseOnLostFocus;
    }

    /**
     * Set the display settings to define the display created. Examples of
     * display parameters include display pixel width and height,
     * color bit depth, z-buffer bits, antialiasing samples, and update freqency.
     *
     * @param settings The settings to set.
     */
    public void setSettings(AppSettings settings){
        this.settings = settings;
        if (context != null && settings.useInput() != inputEnabled){
            // may need to create or destroy input based
            // on settings change
            inputEnabled = !inputEnabled;
            if (inputEnabled){
                initInput();
            }else{
                destroyInput();
            }
        }else{
            inputEnabled = settings.useInput();
        }
    }

    private void initDisplay(){
        // aquire important objects
        // from the context
        settings = context.getSettings();
        timer = context.getTimer();
        renderer = context.getRenderer();
    }

    /**
     * Creates the camera to use for rendering. Default values are perspective
     * projection with 45° field of view, with near and far values 1 and 1000
     * units respectively.
     */
    private void initCamera(){
        cam = new Camera(settings.getWidth(), settings.getHeight());

        cam.setFrustumPerspective(45f, (float)cam.getWidth() / cam.getHeight(), 1f, 1000f);
        cam.setLocation(new Vector3f(0f, 0f, 10f));
        cam.lookAt(new Vector3f(0f, 0f, 0f), Vector3f.UNIT_Y);

        renderManager = new RenderManager(renderer);
        viewPort = renderManager.createMainView("Default", cam);
        guiViewPort = renderManager.createPostView("Gui Default", cam);
        guiViewPort.setClearEnabled(false);
    }

    /**
     * Initializes mouse and keyboard input. Also
     * initializes joystick input if joysticks are enabled in the
     * AppSettings.
     */
    private void initInput(){
        mouseInput = context.getMouseInput();
        if (mouseInput != null)
            mouseInput.initialize();

        keyInput = context.getKeyInput();
        if (keyInput != null)
            keyInput.initialize();

        if (!settings.getBoolean("DisableJoysticks")){
            joyInput = context.getJoyInput();
            if (joyInput != null)
                joyInput.initialize();
        }

        inputManager = new InputManager(mouseInput, keyInput, joyInput);
    }

    /**
     * Initializes the content manager.
     */
    private void initContentManager(){
        manager = G3DSystem.newAssetManager();
    }

    /**
     * @return The content manager for this application.
     */
    public AssetManager getAssetManager(){
        return manager;
    }

    /**
     * @return the input manager.
     */
    public InputManager getInputManager(){
        return inputManager;
    }

    /**
     * @return The renderer for the application, or null if was not started yet.
     */
    public Renderer getRenderer(){
        return renderer;
    }

    /**
     * @return The display context for the application, or null if was not
     * started yet.
     */
    public G3DContext getContext(){
        return context;
    }

    /**
     * @return The camera for the application, or null if was not started yet.
     */
    public Camera getCamera(){
        return cam;
    }
    
    public void start(){
        start(G3DContext.Type.Display);
    }

    /**
     * Starts the application. Creating a display and running the main loop.
     */
    public void start(G3DContext.Type contextType){
        if (context != null && context.isCreated()){
            logger.warning("start() called when application already created!");
            return;
        }

        if (settings == null){
            settings = new AppSettings(true);
        }
        
        logger.fine("Starting application: "+getClass().getName());
        context = G3DSystem.newContext(settings, contextType);
        context.setSystemListener(this);
        context.create();
    }

    public void createCanvas(){
        if (context != null && context.isCreated()){
            logger.warning("createCanvas() called when application already created!");
            return;
        }

        if (settings == null){
            settings = new AppSettings(true);
        }

        logger.fine("Starting application: "+getClass().getName());
        context = G3DSystem.newContext(settings, G3DContext.Type.Canvas);
    }

    public void startCanvas(){
        context.setSystemListener(this);
        context.create();
    }

    public void reshape(int w, int h){
        renderManager.notifyReshape(w, h);
    }

    public void restart(){
        context.restart();
    }

    /**
     * Requests the display to close, shutting down the main loop
     * and making neccessary cleanup operations.
     */
    public void stop(){
        logger.fine("Closing application: "+getClass().getName());
        context.destroy();
    }

    /**
     * Do not call manually.
     * Callback from ContextListener.
     *
     * Initializes the <code>Application</code>, by creating a display and
     * default camera. If display settings are not specified, a default
     * 640x480 display is created. Default values are used for the camera;
     * perspective projection with 45° field of view, with near
     * and far values 1 and 1000 units respectively.
     */
    public void initialize(){
        initDisplay();
        initCamera();
        if (inputEnabled){
            initInput();
        }

        // update timer so that the next delta is not too large
        timer.update();

        // user code here..
    }

    public void handleError(String errMsg, Throwable t){
        if (t != null)
            t.printStackTrace();
    }

    public void gainFocus(){
        if (pauseOnFocus){
            speed = 1;
            context.setAutoFlushFrames(true);
        }
    }

    public void loseFocus(){
        if (pauseOnFocus){
            speed = 0;
            context.setAutoFlushFrames(false);
        }
    }

    public void requestClose(boolean esc){
        context.destroy();
    }

    public <V> Future<V> enqueue(Callable<V> callable) {
        AppTask<V> task = new AppTask<V>(callable);
        taskQueue.add(task);
        return task;
    }

    /**
     * Do not call manually.
     * Callback from ContextListener.
     */
    public void update(){
        AppTask<?> task = taskQueue.poll();
        toploop: do {
            if (task == null) break;
            while (task.isCancelled()) {
                task = taskQueue.poll();
                if (task == null) break toploop;
            }
            task.invoke();
        } while (((task = taskQueue.poll()) != null));
    
        if (speed == 0)
            return;

        timer.update();

        if (inputEnabled){
            if (mouseInput != null)
                mouseInput.update();

            if (keyInput != null)
                keyInput.update();

            if (joyInput != null)
                joyInput.update();

            inputManager.update(timer.getTimePerFrame());
        }

        // user code here..
    }

    protected void destroyInput(){
        if (mouseInput != null)
            mouseInput.destroy();

        if (keyInput != null)
            keyInput.destroy();

        if (joyInput != null)
            joyInput.destroy();

        inputManager = null;
    }

    /**
     * Do not call manually.
     * Callback from ContextListener.
     */
    public void destroy(){
        destroyInput();
        timer.reset();
        renderer.cleanup();
    }

}
