package com.jmex.jbullet.debug;

import com.jme.input.KeyBindingManager;
import com.jme.input.KeyInput;
import com.jme.system.DisplaySystem;
import com.jmex.game.state.DebugGameState;

/**
 * Specialisation on the DebugGame state to provide the option of rendering the details of the physics simulation.
 *  <p/>
 *  With default key binding the option to rotate through the various render choices is done via F5 key 'F5'.
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
    /** String identifier, used when checking whether user triggered the bound input (F5).*/
    private static final String DRAW_STATE_SWITCH_COMMAND = "physics_switch_between_draw_states";

    /**
     *  Provides the various draw states avaiiable to the PhysicsDebugGameState, as the option
     *  to not render the JME scene is available.
     */
    public enum DrawState
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
        private static DrawState nextDrawState( DrawState current )
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

        // Start off without the physics being drawn
        drawState = DrawState.NoPhysicsDebug;
    }

    /**
     *  Create the key bindings that will be listened for to switch between which physics elements are drawn.
     */
    private void setupPhysicsKeyBindings()
    {
        final int drawStateSwitchKey = KeyInput.KEY_F5;
        KeyBindingManager.getKeyBindingManager().set( DRAW_STATE_SWITCH_COMMAND, drawStateSwitchKey );
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
        else
        {
            // Render the text node anyway (it's part of: super.super.render() which is not being invoked)
            DisplaySystem.getDisplaySystem().getRenderer().draw( textNode );
        }

        // Draw the physics wireframes?
        if ( drawState.drawPhysicsScene )
        {
            PhysicsDebugger.drawWireframes( DisplaySystem.getDisplaySystem().getRenderer() );
        }
    }

    /**
     *  Changes the existing key binding for the switching between draw states to 
     *  that given.
     *
     * @param switchKey the new key binding. WARNING: no checks are made on the
     *      given int, or whether an existing command is already bound to that key -
     *      so be careful!
     */
    public void setStateDrawingSwitchKey( int switchKey )
    {
        // Remove the existing binding, then Set the new one
        KeyBindingManager.getKeyBindingManager().remove( DRAW_STATE_SWITCH_COMMAND );
        KeyBindingManager.getKeyBindingManager().set( DRAW_STATE_SWITCH_COMMAND, switchKey );
    }

    /**
     *  Sets the current draw state to that given.
     *  <p/>
     *  The draw state determines whether the the JME scene is rendered and
     *  whether the PhysicsBounds scene is rendered.
     *
     * @param desired the desired state, should not be <code>null</code>.
     */
    public void setDrawState( DrawState desired )
    {
        assert desired != null : "You are not allowed to set null as the DrawState";
        drawState = desired;
    }
}