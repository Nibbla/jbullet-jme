package com.jmex.jbullet.debug;

import com.jme.input.KeyBindingManager;
import com.jme.input.KeyInput;
import com.jme.system.DisplaySystem;
import com.jmex.game.state.DebugGameState;

/**
 * Specialisation on the DebugGame state to provide the option of rendering the details of the physics simulation.
 *  <p/>
 *  With default key binding the option to rotate through the various render choices is done via backslash key '\'.
 *  <ul>
 *      <li>Render the both JME scene and the physics debug scene.</li>
 *      <li>Render the only the physics debug scene.</li>
 *      <li>Render only the JME scene.</li>
 * </ul>
 *
 * @author CJ Hare
 */
public class PhysicsDebugGameState extends DebugGameState
{
    private static final String DRAW_STATE_SWITCH_COMMAND = "physics_switch_between_draw_states";

    /**
     *  Provides the various draw states avaiiable to the PhysicsDebugGameState, as the option
     *  to not render the JME scene is available.
     */
    private enum DrawState
    {
        Both( true, true ),
        PhysicsDebugOnly( true, false ),
        NoPhysicsDebug( false, true );

        public final boolean drawPhysicsScene;

        public final boolean drawJmeScene;

        DrawState( boolean drawPhysicsScene, boolean drawJmeScene )
        {
            this.drawPhysicsScene = drawPhysicsScene;
            this.drawJmeScene = drawJmeScene;
        }

        /**
         *  Switches the states between the current options:
         *  <ul>
         *      <li>Both : PhysicsDebug with JmeScene</li>
         *      <li>PhysicsDebugOnly: Physics Debug, no JmeScene</li>
         *      <li>NoPhysicsDebug: Only the normal JmeScene</li>
         *  </ul>
         *
         *  @param current the current draw state.
         *  @return the next draw state combination.
         */
        public static DrawState nextDrawState( DrawState current )
        {
            DrawState nextState = null;

            switch ( current )
            {
                case Both:
                    nextState = PhysicsDebugOnly;
                    break;
                case PhysicsDebugOnly:
                    nextState = NoPhysicsDebug;
                    break;
                case NoPhysicsDebug:
                    nextState = Both;
                    break;
                default:
                    throw new IllegalStateException( "Given a DrawState that is uncatered for" );
            }

            return nextState;
        }
    }
    /** The state is used to decided what will be drawn in the render().*/
    private DrawState drawState;

    /**
     *  Creates the PhysicsDebugGameState that handles input in the same ways as the default DebugGameState.
     */
    public PhysicsDebugGameState()
    {
        super( true );

        // Create the keybinds to switch between the render states for phyiscs rendering
        setupPhysicsKeyBindings();

        // Force the wireframe drawing of the DebugGameState - DrawState.JmeScene
        drawState = DrawState.Both;
        wireState.setEnabled( true );
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
            drawState = DrawState.nextDrawState( drawState );

            rootNode.updateRenderState();
        }
    }

    @Override
    public void render( float tpf )
    {
        // Do we render the JME scene?
        if ( drawState.drawJmeScene )
        {
            super.render( tpf );
        }

        // Draw the physics wireframes?
        if ( drawState.drawPhysicsScene )
        {
            PhysicsDebugger.drawWireframes( DisplaySystem.getDisplaySystem().getRenderer() );
        }
    }
}
