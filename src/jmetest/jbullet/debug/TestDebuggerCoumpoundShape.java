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

import com.jme.bounding.BoundingBox;
import com.jme.math.FastMath;
import com.jme.math.Quaternion;
import com.jme.math.Vector3f;
import com.jme.scene.Node;
import com.jme.scene.Spatial;
import com.jme.scene.shape.Box;
import com.jme.scene.shape.Cylinder;
import com.jme.scene.shape.Sphere;
import com.jme.util.GameTaskQueueManager;
import com.jmex.editors.swing.settings.GameSettingsPanel;
import com.jmex.game.StandardGame;
import com.jmex.game.state.GameStateManager;
import com.jmex.jbullet.PhysicsSpace;
import com.jmex.jbullet.collision.shapes.BoxCollisionShape;
import com.jmex.jbullet.collision.shapes.CapsuleCollisionShape;
import com.jmex.jbullet.collision.shapes.CollisionShape;
import com.jmex.jbullet.collision.shapes.CompoundCollisionShape;
import com.jmex.jbullet.collision.shapes.CylinderCollisionShape;
import com.jmex.jbullet.collision.shapes.SphereCollisionShape;
import com.jmex.jbullet.debug.PhysicsDebugGameState;
import com.jmex.jbullet.nodes.PhysicsNode;
import com.jmex.jbullet.util.PhysicsUtil;

/**
 * Testing out whether the PhysicsDebug view can correctly render CompoundCollisionShapes.
 *
 * @author CJ Hare
 */
public class TestDebuggerCoumpoundShape
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

        state.setText( "A compound collision shape made from every primative" );

        // Only render the physics bounds
        state.setDrawState( PhysicsDebugGameState.DrawState.Both );

        PhysicsNode compound = createCompoundCollisionShape();
        compound.setLocalTranslation( 0f, 0f, 0f );
        state.getRootNode().attachChild( compound );
        compound.updateRenderState();
        pSpace.add( compound );

        // The floor, which does not move (mass=0)
        Spatial box = new Box( "physicsfloor", Vector3f.ZERO, 20f, 0.2f, 20f );
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
        StandardGame game = new StandardGame( "Test the rendering of a Compound Collision Shape" );
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

    private static PhysicsNode createCompoundCollisionShape()
    {
        Node compoundSpatial = new Node();
        CompoundCollisionShape compoundShape = new CompoundCollisionShape();

        // The primative shape: Box
        Box box = new Box( "Box", Vector3f.ZERO, 1f, 1f, 1f );
        box.setLocalTranslation( -3f, 0f, 0f );
        box.setLocalRotation( new Quaternion().fromAngleAxis( FastMath.QUARTER_PI, Vector3f.UNIT_Y ) );
        box.setModelBound( new BoundingBox() );
        box.updateModelBound();
        compoundSpatial.attachChild( box );
        BoxCollisionShape boxShape = new BoxCollisionShape( new Vector3f( 1f, 1f, 1f ) );
        compoundShape.addChildShape( boxShape, box.getLocalTranslation(), box.getLocalRotation().toRotationMatrix() );

        // The primative shape: Capsule
        Sphere capsule = new Sphere( "Capsule as Sphere", Vector3f.ZERO, 32, 32, 1f );
        capsule.setLocalTranslation( 2f, -1f, 0f );
        capsule.setModelBound( new BoundingBox() );
        capsule.updateModelBound();
        compoundSpatial.attachChild( capsule );
        CapsuleCollisionShape capsuleShape = new CapsuleCollisionShape( 1f, 1f );
        compoundShape.addChildShape( capsuleShape, capsule.getLocalTranslation() );

        // The primative shape: Cyclinder
        Cylinder cylinder = new Cylinder( "Cylinder", 32, 32, 2f, 5f, true );
        cylinder.setLocalTranslation( 0f, 1f, 1f );
        cylinder.setLocalRotation( new Quaternion().fromAngleAxis( FastMath.QUARTER_PI, Vector3f.UNIT_X ) );
        cylinder.setModelBound( new BoundingBox() );
        cylinder.updateModelBound();
        compoundSpatial.attachChild( cylinder );
        CylinderCollisionShape cylinderShape = new CylinderCollisionShape( new Vector3f( 2f, 5f, 2.5f ) );
        compoundShape.addChildShape( cylinderShape, cylinder.getLocalTranslation(), cylinder.getLocalRotation().toRotationMatrix() );

        // The primative shape: Sphere
        Sphere sphere = new Sphere( "Sphere", Vector3f.ZERO, 32, 32, 3f );
        sphere.setLocalTranslation( -4f, 3f, -2f );
        sphere.setModelBound( new BoundingBox() );
        sphere.updateModelBound();
        compoundSpatial.attachChild( sphere );
        SphereCollisionShape sphereShape = new SphereCollisionShape( 3f );
        compoundShape.addChildShape( sphereShape, sphere.getLocalTranslation() );

        // Make a moving compound rigid body collision physics node
        PhysicsNode compound = new PhysicsNode( compoundSpatial, compoundShape );
        compound.setKinematic( true );

        return compound;
    }
}
