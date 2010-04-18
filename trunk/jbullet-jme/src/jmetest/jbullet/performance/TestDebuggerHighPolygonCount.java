package jmetest.jbullet.performance;

import com.jme.math.Vector3f;
import com.jme.scene.shape.Box;
import com.jme.scene.shape.Sphere;
import com.jme.util.GameTaskQueueManager;
import com.jmex.editors.swing.settings.GameSettingsPanel;
import com.jmex.game.StandardGame;
import com.jmex.game.state.DebugGameState;
import com.jmex.game.state.GameStateManager;
import com.jmex.jbullet.PhysicsSpace;
import com.jmex.jbullet.collision.shapes.CollisionShape;
import com.jmex.jbullet.debug.PhysicsDebugGameState;
import com.jmex.jbullet.nodes.PhysicsNode;
import java.util.concurrent.Callable;

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
        DebugGameState state = new PhysicsDebugGameState()
        {
            @Override
            public void update( float tpf )
            {
                super.update( tpf );
                pSpace.update( tpf );
            }
        };

        state.setText( "The UV Sphere has 128 zSamples and 128 radialSamples ~ 32K triangles" );

        // High poly count shape
        Sphere highPoly = new Sphere( "SphereShape", Vector3f.ZERO, 128, 128, 4f );
        // Using a MESH creates a physics object with the same number of triangles as the shape
        PhysicsNode highPolyNode = new PhysicsNode( highPoly, CollisionShape.ShapeTypes.MESH );
        highPolyNode.setLocalTranslation( 0, -1, 0 );
        highPolyNode.updateRenderState();
        state.getRootNode().attachChild( highPolyNode );
        pSpace.add( highPolyNode );

        // The floor, which does not move (mass=0)
        PhysicsNode floor = new PhysicsNode( new Box( "physicsfloor", Vector3f.ZERO, 20f, 0.2f, 20f ), CollisionShape.ShapeTypes.BOX, 0 );
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
