/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */

package com.jmex.jbullet;

import com.g3d.app.Application;
import java.util.Timer;
import java.util.logging.Level;
import java.util.logging.Logger;

/**
 *
 * @author normenhansen
 */
public class PhysicsApplication extends Application implements Runnable{
    private Thread physicsThread;
    private PhysicsSpace pSpace;

    private boolean multithreaded=true;
    private boolean running=false;

    private long lastTime=-1;

    private float accuracy=1/60f;

    public PhysicsApplication() {
        super();
    }

    public void startPhysics(){
        if(!multithreaded){
            pSpace=new PhysicsSpace();
        }
        else{
            startThread();
            while(pSpace==null){
                try {
                    Thread.sleep(1);
                } catch (InterruptedException ex) {
                    Logger.getLogger(PhysicsApplication.class.getName()).log(Level.SEVERE, null, ex);
                }
            }
        }
    }

    @Override
    public void update() {
        super.update();
        if(!multithreaded){
            physicsUpdate(timer.getTimePerFrame());
        }
    }

    @Override
    protected void finalize() throws Throwable {
        super.finalize();
    }

    @Override
    public void destroy() {
        super.destroy();
        if(physicsThread!=null){
            try {
                running = false;
                physicsThread.join();
            } catch (InterruptedException ex) {
                Logger.getLogger(PhysicsApplication.class.getName()).log(Level.SEVERE, null, ex);
            }
        }
    }

    /**
     * can be overridden by user, called from physics thread!
     */
    public void physicsUpdate(float tpf){
        if(pSpace==null) return;
        pSpace.update(tpf);
    }

    private void startThread(){
        running=true;
        if(physicsThread!=null)
            return;
        physicsThread=new Thread(this);
        physicsThread.start();
    }

    public void run() {
        pSpace=new PhysicsSpace();
        while(running){
            lastTime=System.currentTimeMillis();
            physicsUpdate(accuracy);
            float wait=(lastTime+(accuracy*1000))-System.currentTimeMillis();
            if(wait<=0){
                try {
                    //                    System.out.println("sleep "+wait);
                    Thread.sleep(Math.round(accuracy * 1000));
                } catch (InterruptedException ex) {
                    Logger.getLogger(PhysicsApplication.class.getName()).log(Level.SEVERE, null, ex);
                }
            }
            else{
                try {
                    System.out.println("sleep "+wait);
                    Thread.sleep(Math.round(wait));
                } catch (InterruptedException ex) {
                    Logger.getLogger(PhysicsApplication.class.getName()).log(Level.SEVERE, null, ex);
                }
            }
        }
    }

    public PhysicsSpace getPhysicsSpace() {
        return pSpace;
    }

}
