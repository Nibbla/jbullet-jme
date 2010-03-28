package com.jmex.jbullet.debug;

import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.linearmath.Transform;
import com.jme.input.KeyBindingManager;
import com.jme.input.KeyInput;
import com.jmex.game.state.DebugGameState;
import com.jmex.jbullet.PhysicsSpace;

/**
 * Specialisation on the DebugGame state to provide the option of rendering the details of the physics simulation.
 *
 * @author CJ Hare
 */
public class PhysicsDebugGameState extends DebugGameState
{
    private static final String DRAW_STATE_SWITCH_COMMAND = "physics_switch_between_draw_states";

    /**
     *  Store for the various render options that the PhysicsDebugGameState can switch between.
     */
    private enum DrawState
    {
        PhysicsWireframe( true ),
        JmeWireframe( true );

        public boolean draw;

        DrawState( boolean drawByDefault )
        {
            this.draw = drawByDefault;
        }

        /**
         *  Switches the states between the current options:
         *  <ul>
         *      <li>PhysicsWireframe, no JmeWireframe</li>
         *      <li>PhysicsWireframe with JmeWireframe</li>
         *      <li>No PhysicsWireframe with JmeWireframe</li>
         *  </ul>
         */
        public static void nextDrawState()
        {
            // Figure out the current state
            if ( PhysicsWireframe.draw )
            {
                if ( JmeWireframe.draw )
                {
                    // Move to state: 'No wireframe with Jme scene'
                    PhysicsWireframe.draw = false;
                }
                else
                {
                    // Move to state: 'Wireframes with Jme scene'
                    JmeWireframe.draw = true;
                }
            }
            else
            {
                // Move to state: 'Wireframes, no JmeScene'
                PhysicsWireframe.draw = true;
                JmeWireframe.draw = false;
            }
        }
    }

    /**
     *  Creates the PhysicsDebugGameState that handles input in the same ways as the default DebugGameState.
     */
    public PhysicsDebugGameState()
    {
        super( true );

        // Create the keybinds to switch between the render states for phyiscs rendering
        setupPhysicsKeyBindings();

        // Force the wireframe drawing of the DebugGameState - DrawState.JmeWireframe
        wireState.setEnabled( DrawState.JmeWireframe.draw );
    }

    /**
     *  Create the key bindings that will be listened for to switch between which physics elements are drawn.
     */
    private void setupPhysicsKeyBindings()
    {
        /** Assign key '/' to action "physics_switch_between_states". */
        KeyBindingManager.getKeyBindingManager().set( DRAW_STATE_SWITCH_COMMAND, KeyInput.KEY_BACKSLASH );
    }

    @Override
    public void update( float tpf )
    {
        super.update( tpf );

        // Check for the application being paused
        if ( pause )
        {
            return;
        }

        // Test whether the key to switch between draw states is pressed
        if ( KeyBindingManager.getKeyBindingManager().isValidCommand(
                DRAW_STATE_SWITCH_COMMAND, false ) )
        {
            DrawState.nextDrawState();

            // Force the wireframe drawing of the DebugGameState
            wireState.setEnabled( DrawState.JmeWireframe.draw );

            rootNode.updateRenderState();
        }
    }

    @Override
    public void render( float tpf )
    {
        super.render( tpf );

        // Draw the physics wireframes?
        if ( DrawState.PhysicsWireframe.draw )
        {
//            		DisplaySystem.getDisplaySystem().getRenderer().draw()

            drawPhysicsElements();
        }
    }
    //TODO convert below this line to use only JME rendering?
    /*
     *  Everything below here is tied to rendering using the Lwjgl copied from bullet.
     */
    private final Transform transform = new Transform();

    private final javax.vecmath.Vector3f wireColour = new javax.vecmath.Vector3f();

    private static final LwjglGL gl = new LwjglGL();

    public void drawPhysicsElements()
    {
        DynamicsWorld dynamicsWorld = PhysicsSpace.getPhysicsSpace().getDynamicsWorld();

        if ( dynamicsWorld != null )
        {
            // Red colour
            wireColour.set( 1f, 0f, 0f );

            for ( CollisionObject collisionObject : dynamicsWorld.getCollisionObjectArray() )
            {
                collisionObject.getWorldTransform( transform );

                //TODO remove the below code, as setting as identity removed rotations!

                //TODO figure out why the world transform for PhysicsCharacterNodes is wrong
                // A hack to get around the PhysicsCharacter node having a blank basis (scaling)
               /* float x = transform.origin.x;
                float y = transform.origin.y;
                float z = transform.origin.z;

                transform.setIdentity();

                transform.origin.x = x;
                transform.origin.y = y;
                transform.origin.z = z;
                 */

                // Yellow Colour
                wireColour.set( 1f, 1f, 0f ); // wants deactivation

                // Is the object is active
                if ( collisionObject.getActivationState() == 1 ) // active
                {
                    // Cyan Colour
                    wireColour.set( 0f, 1f, 1f );
                }
                // Is the object asleep / deactivated
                else if ( collisionObject.getActivationState() == 2 ) // ISLAND_SLEEPING
                {
                    // Light Blue Colour
                    wireColour.set( 0.5f, 0.5f, 1f );
                }

                GLShapeDrawer.drawWireframeObject( gl, transform, collisionObject.getCollisionShape(), wireColour );
            }
        }
    }
}
