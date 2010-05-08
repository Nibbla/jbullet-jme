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

import com.jme.math.Quaternion;
import com.jme.renderer.Renderer;
import javax.vecmath.Vector3f;

/**
 *  The behavioural contract for a wireframe of a Rigid Body shape from bullet physics.
 *
 * @author CJ Hare
 */
public interface RigidBodyWireframe
{
    /**
     *  Ensures the wireframe's world translation matches that given.
     *
     * @param bulletTranslation the coordinates of the wireframe's paired broadphase rigid body from the bullet physics simulation.
     */
    public void updateWorldTranslation( Vector3f bulletTranslation );

    /**
     * Ensures the wireframe's world rotation matches that given.
     *
     * @param bulletRotation the rotation of the wireframe's paired broadphase rigid body from the bullet physics simulation.
     */
    public void updateWorldRotation( Quaternion bulletRotation );

    /**
     *  Ensure that frustum culling test is performed prior to rendering.
     *
     * @param renderer the jme renderer to draw the rigid body wireframe onto (if it's in view).
     */
    public void render( Renderer renderer );
}
