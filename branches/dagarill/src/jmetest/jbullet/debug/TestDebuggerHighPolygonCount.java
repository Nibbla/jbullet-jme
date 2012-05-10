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
package jmetest.jbullet.debug;

import java.util.concurrent.Callable;

import com.jme.math.Vector3f;
import com.jme.scene.Spatial;
import com.jme.scene.shape.Box;
import com.jme.scene.shape.Sphere;
import com.jme.util.GameTaskQueueManager;
import com.jmex.editors.swing.settings.GameSettingsPanel;
import com.jmex.game.StandardGame;
import com.jmex.game.state.GameStateManager;
import com.jmex.jbullet.PhysicsSpace;
import com.jmex.jbullet.collision.shapes.BoxCollisionShape;
import com.jmex.jbullet.collision.shapes.CollisionShape;
import com.jmex.jbullet.collision.shapes.MeshCollisionShape;
import com.jmex.jbullet.debug.PhysicsDebugGameState;
import com.jmex.jbullet.nodes.PhysicsNode;
import com.jmex.jbullet.util.PhysicsUtil;

/**
 *  Test to ensure rendering performance of the Physics Debugger is on par with JME.
 *  <p/>
 *  This tests few shapes with a high polygon count.
 *  <p/>
 *  Ideally there should be no framerate difference between when only the physics bound
 *  is rendering and only the JME scene is being rendered (as both contain the same objects
 *  and triangles).
 *
 * @author CJ Hare
 */
public class TestDebuggerHighPolygonCount
{
    public static void setupGame()
    {
        // creates and initializes the PhysicsSpace
        final PhysicsSpace pSpace = PhysicsSpace.getPhysicsSpace( PhysicsSpace.BroadphaseTypes.AXIS_SWEEP_3 );

        // Create a DebugGameState
        // - override the update method to update/sync physics space
        PhysicsDebugGameState state = new PhysicsDebugGameState()
        {
            @Override
            public void update( float tpf )
            {
                super.update( tpf );
                pSpace.update( tpf );
            }
        };

        state.setText( "Each UV Sphere has 256 zSamples and 256 radialSamples ~ 130K triangles (~130K in bullet)" );

        // Only render the physics bounds
        state.setDrawState( PhysicsDebugGameState.DrawState.PhysicsDebugOnly );

        // High poly count shape
        Sphere highPoly = new Sphere( "SphereShape", Vector3f.ZERO, 256, 256, 8f );
        // Using a MESH creates a physics object with the same number of triangles as the shape
        PhysicsNode highPolyNode = new PhysicsNode( highPoly, PhysicsUtil.generateCollisionShape(highPoly, MeshCollisionShape.class), 0f );
        highPolyNode.setLocalTranslation( 0, 0, 0 );
        highPolyNode.updateRenderState();
        state.getRootNode().attachChild( highPolyNode );
        pSpace.add( highPolyNode );

        // The floor, which does not move (mass=0)
        Spatial box =  new Box( "physicsfloor", Vector3f.ZERO, 20f, 0.2f, 20f );
        PhysicsNode floor = new PhysicsNode(box, PhysicsUtil.generateCollisionShape(box, BoxCollisionShape.class), 0 );
        floor.setLocalTranslation( new Vector3f( 0f, -6, 0f ) );
        state.getRootNode().attachChild( floor );
        floor.updateRenderState();
        pSpace.add( floor );

        // Add the gamestate to the manager
        GameStateManager.getInstance().attachChild( state );

        // Activate the game state, so we can see it
        state.setActive( true );
    }

    public static void main( String[] args ) throws Exception
    {
        // Enable statistics gathering
        System.setProperty( "jme.stats", "set" );

        // Instantiate StandardGame
        StandardGame game = new StandardGame( "Performance Test: High poly count" );
        // Show settings screen
        if ( GameSettingsPanel.prompt( game.getSettings() ) )
        {
            // Start StandardGame, it will block until it has initialized successfully, then return
            game.start();

            GameTaskQueueManager.getManager().update( new Callable<Void>()
            {
                @Override
                public Void call() throws Exception
                {
                    setupGame();
                    return null;
                }
            } );
        }
    }
}
