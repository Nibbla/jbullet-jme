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
package com.jmex.jbullet.debug;

import com.bulletphysics.collision.dispatch.CollisionObject;
import com.jme.renderer.ColorRGBA;
import com.jme.scene.state.MaterialState;
import com.jme.system.DisplaySystem;

/**
 *  The available bullet states that rigid body can be in at any time.
 *  <p/>
 *  This relates in to the combinations of activationState and collisionFlags.
 *
 * @author CJ Hare
 */
public enum WireframeActivityState
{
    /**
     * Static object (mass == 0f): bullet does not query for translation / rotation.
     */
    Static( ColorRGBA.red ),
    /**
     * Kinematic object: bullet retrieves the translation / rotation every update,
     * bullet does not include in Forward Dynamics (object movement cause by physics simulation).
     */
    Kinematic( ColorRGBA.blue ),
    /**
     * Character object, (ghost) bullet controls object movement.
     */
    Character( ColorRGBA.green ),
    /**
     * Dynamic object: bullet controls object movement.<br/>
     *  The object's forces are above the sleep threshold so movement is being calculated every update.
     */
    DynamicAwake( ColorRGBA.yellow ),
    /**
     * Dynamic object: bullet controls object movement.<br/>
     *  The object's forces are below the sleep threshold, marked for sleep, however movement is still being calculated every update.
     */
    DynamicSleepy( ColorRGBA.cyan ),
    /**
     * Dynamic object: bullet controls object movement.<br/>
     *  The object's forces are below the sleep threshold, movement is not being calculated.
     */
    DynamicAsleep( ColorRGBA.magenta ),
    /**
     * None of the above states.
     */
    Default( ColorRGBA.white );

    public final MaterialState colour;

    WireframeActivityState( ColorRGBA stateColour )
    {
        colour = DisplaySystem.getDisplaySystem().getRenderer().createMaterialState();
        colour.setDiffuse( stateColour );
        colour.setAmbient( stateColour );
        colour.setEmissive( stateColour );
        colour.setSpecular( stateColour );
        colour.setEnabled( true );
        colour.setMaterialFace( MaterialState.MaterialFace.FrontAndBack );
    }

    /**
     *  Retrieves the appropiate ActivityState for the given CollisionObject based on it's collisionFlag and activationState.
     *
     * @param collisionObect the object who is to have a matching state.
     * @return the wireframe state appropiate for the given collision object, never <code>null</code>
     */
    public static WireframeActivityState getActivityState( CollisionObject collisionObect )
    {
        WireframeActivityState state = null;

        // Type of collision object (as a flag)
        final int collisionFlag = collisionObect.getCollisionFlags();

        // Are we Kinematic?
        if ( CollisionFlags.isOfType( CollisionFlags.Kinematic, collisionFlag ) )
        {
            state = Kinematic;
        }
        // Or are we a Character?
        else if ( CollisionFlags.isOfType( CollisionFlags.Character, collisionFlag ) )
        {
            state = Character;
        }
        // Are we a static object?
        else if ( CollisionFlags.isOfType( CollisionFlags.Static, collisionFlag ) )
        {
            state = Static;
        }
        else
        {
            /*
             * What we know:
             *      - not static
             *      - not character
             *      - not kinematic
             */
            switch ( ActivationState.getState( collisionObect.getActivationState() ) )
            {
                case Active:
                    state = DynamicAwake;
                    break;
                case IslandSleeping:
                    state = DynamicAsleep;
                    break;
                case WantsDeactivation:
                    state = DynamicSleepy;
                    break;
                default:
                    state = Default;
                    break;
            }
        }

        assert state != null : "We should not be getting a null for the ActivityState - bad programmer!";

        return state;
    }
}

/**
 * Activation state values.
 */
enum ActivationState
{
    Active( 1 ),
    IslandSleeping( 2 ),
    WantsDeactivation( 3 ),
    DisableDeactivation( 4 ),
    DisableSimulation( 5 );

    public final int value;

    ActivationState( int value )
    {
        this.value = value;
    }

    public static ActivationState getState( int bulletActivationState )
    {
        ActivationState state = null;

        switch ( bulletActivationState )
        {
            case 1:
                state = Active;
                break;
            case 2:
                state = IslandSleeping;
                break;
            case 3:
                state = WantsDeactivation;
                break;
            case 4:
                state = DisableDeactivation;
                break;
            case 5:
                DisableDeactivation:
                break;
            default:
                throw new IllegalArgumentException( "Value not catered for: " + bulletActivationState );
        }

        return state;
    }
}

/**
 * Bullet's collision flags assigned to it's objects / what kind of collision object.
 */
enum CollisionFlags
{
    Static( 1 ),
    Kinematic( 2 ),
    NoContactResponse( 4 ),
    CustomMaterialCallback( 8 ),
    Character( 16 );

    public final int flag;

    CollisionFlags( int flag )
    {
        this.flag = flag;
    }

    /**
     *  Tests whether the given collision flag matched the given type of flag.
     *
     * @param type the type of flag to test for / expected.
     * @param collisionFlag the value (flag) to test.
     * @return <code>true</code> the given collisionFlag contains the given type.
     */
    public static boolean isOfType( CollisionFlags type, int collisionFlag )
    {
        return (collisionFlag & type.flag) != 0;
    }
}
