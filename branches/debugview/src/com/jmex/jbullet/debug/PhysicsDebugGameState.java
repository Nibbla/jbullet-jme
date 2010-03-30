package com.jmex.jbullet.debug;

import com.bulletphysics.collision.dispatch.CollisionObject;
import com.bulletphysics.collision.shapes.CollisionShape;
import com.bulletphysics.collision.shapes.ConcaveShape;
import com.bulletphysics.collision.shapes.ConvexShape;
import com.bulletphysics.collision.shapes.PolyhedralConvexShape;
import com.bulletphysics.dynamics.DynamicsWorld;
import com.bulletphysics.linearmath.Transform;
import com.jme.input.KeyBindingManager;
import com.jme.input.KeyInput;
import com.jme.renderer.Renderer;
import com.jme.system.DisplaySystem;
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
        PhysicsDebug( true ),
        JmeScene( true );

        public boolean draw;

        DrawState( boolean drawByDefault )
        {
            this.draw = drawByDefault;
        }

        /**
         *  Switches the states between the current options:
         *  <ul>
         *      <li>PhysicsDebug, no JmeScene</li>
         *      <li>PhysicsDebug with JmeScene</li>
         *      <li>No PhysicsDebug with JmeScene</li>
         *  </ul>
         */
        public static void nextDrawState()
        {
            // Figure out the current state
            if ( PhysicsDebug.draw )
            {
                if ( JmeScene.draw )
                {
                    // Move to state: 'No wireframe with Jme scene'
                    PhysicsDebug.draw = false;
                }
                else
                {
                    // Move to state: 'Wireframes with Jme scene'
                    JmeScene.draw = true;
                }
            }
            else
            {
                // Move to state: 'Wireframes, no JmeScene'
                PhysicsDebug.draw = true;
                JmeScene.draw = false;
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

        // Force the wireframe drawing of the DebugGameState - DrawState.JmeScene
        wireState.setEnabled( DrawState.JmeScene.draw );
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
            wireState.setEnabled( DrawState.JmeScene.draw );

            rootNode.updateRenderState();
        }
    }

    @Override
    public void render( float tpf )
    {
        // Do we render the JME scene?
        if ( DrawState.JmeScene.draw )
        {
            super.render( tpf );
        }

        // Draw the physics wireframes?
        if ( DrawState.PhysicsDebug.draw )
        {
            drawWireframes( DisplaySystem.getDisplaySystem().getRenderer() );
        }
    }

    public void drawWireframes( Renderer renderer )
    {
        DynamicsWorld dynamicsWorld = PhysicsSpace.getPhysicsSpace().getDynamicsWorld();

        if ( dynamicsWorld != null )
        {
            for ( CollisionObject collisionObject : dynamicsWorld.getCollisionObjectArray() )
            {
                // Get the world translation of the object - determines translation, rotation and scale
                collisionObject.getWorldTransform( transform );

                CollisionShape shape = collisionObject.getCollisionShape();

                //TODO shapes can be polyhedral as well as being other types - the convex & concave shapes have access
                // indicies and vertices
                if ( shape.isConvex() )
                {
                    // Assert the CollisionShape is of the appropiate type
                    assert shape instanceof ConvexShape : "Expecting CollisionShape to be a ConvexShape";
                    ConvexShape convexShape = (ConvexShape) shape;

                    PhysicsDebugger.drawWireframe( renderer, convexShape, transform );
                }
                else if ( shape.isConcave() )
                {
                    // Assert the CollisionShape is of the appropiate type
                    assert shape instanceof ConcaveShape : "Expecting CollisionShape to be a ConcaveShape";
                    ConcaveShape concaveShape = (ConcaveShape) shape;

                    PhysicsDebugger.drawWireframe( renderer, concaveShape, transform );
                }
                else if ( shape.isPolyhedral() )
                {
                    //TODO why bother with polyhedral,  as it's a child of ConvexShape

                    // Assert the CollisionShape is of the appropiate type
                    assert shape instanceof PolyhedralConvexShape : "Expecting CollisionShape to be a PolyhedralConvexShape";
                    PolyhedralConvexShape polyhedralShape = (PolyhedralConvexShape) shape;

                    PhysicsDebugger.drawWireframe( renderer, polyhedralShape, transform );
                }
                //TODO deal with infinite shape?
                //TODO what happens to composite shapes?
            }
        }
    }
    //TODO commnet
    private final Transform transform = new Transform();

}
