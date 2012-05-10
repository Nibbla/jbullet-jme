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
package com.jmex.jbullet.nodes;

import com.jme.math.Quaternion;
import com.jme.math.Vector3f;
import com.jme.scene.Spatial;
import com.jmex.jbullet.collision.shapes.CollisionShape;

/**
 * <p>
 * This class extends the PhysicsCharacterNode functionality to rotate the
 * associated spatial.
 * </p>
 * <p>
 * This class provides two types of rotations mode, by default, using the
 * methods <code>turnLeft(),
 * turnRight(), moveForward and moveBackward</code>, the spatial will be move in
 * absolute mode about map, calling the method
 * <code>turnLeft() or turnRight()</code> the object will move to the left of
 * the map and the associated spatial is rotated automatically in the desired
 * direction
 * </p>
 * 
 * <p>
 * The other mode is activated with the method call:
 * <code>setRotationEnabled(true)</code>, in this case, the
 * <code>turnRight()</code> and </code>turnLeft</code> methods rotates the
 * associated spatial about the Y axis, when you call
 * <code>moveBackward()</code> or <code>moveForward</code> the object will move
 * in the direction where the spatial look.
 * </p>
 * 
 * <p>
 * <b>Important</b>: Don't allow repeats in your Keyboard controller for handle
 * this actions, except when the default rotation mode has been changed, in this
 * case, allow repeats only for <code>turnLeft()</code> and
 * </code>turnRight()</code> actions methods.
 * </p>
 * 
 * @author David García Illanas
 * 
 */
public class PhysicsCharacterRotationNode extends PhysicsCharacterNode {

	/** The direction vector. */
	private final Vector3f walkDirection = new Vector3f();
	/**
	 * Flag for know the actual rotation mode (map absolute rotation by
	 * default).
	 */
	private boolean rotationEnabled = false;
	/** The default rotation speed when the default rotation mode changes. */
	private float rotationSpeed = 0.1f;
	
	/** Amount of rotation when user turn left.*/
	private Quaternion rotationAmountLeft = null;
	/** Amount of rotation when user turn right.*/
	private Quaternion rotationAmountRight = null;
	/** Left direction vector for absolute mode rotations.*/
	private static final Vector3f TURN_LEFT = new Vector3f(-.1f, 0, 0);
	/** Right direction vector for absolute mode rotations.*/
	private static final Vector3f TURN_RIGHT = new Vector3f(.1f, 0, 0);
	/** Forward direction vector for absolute mode rotations.*/
	private static final Vector3f MOVE_FORWARD = new Vector3f(0, 0, -.1f);
	/** Backward direction vector for absolute mode rotations.*/
	private static final Vector3f MOVE_BACKWARD = new Vector3f(0, 0, .1f);
	
	/** Aux vector for optimize the walkDirection call when default rotation mode change.*/
	private Vector3f walkDirectionAux = null;
	/** Aux quaternion for optimize the rotation of spatial in the default rotation mode.*/
	private Quaternion rotationAux = new Quaternion();

	/**
	 * Build the object
	 * 
	 * @param spat
	 *            the associated Spatial, for make the rotations.
	 * @param shape
	 *            the collisionShape for the physics, typically
	 *            <code>SphereCollisionShape</code> instance
	 * @param stepHeight
	 */
	public PhysicsCharacterRotationNode(Spatial spat, CollisionShape shape, float stepHeight) {
		super(spat, shape, stepHeight);
	}

	/**
	 * Moves the spatial to the left of the map (default rotation mode) or
	 * rotate the spatial to the left (rotationEnabled mode).
	 * 
	 * @param active
	 *            is the action key pressed?
	 */
	public void turnLeft(boolean active) {
		if (rotationEnabled) {
			if (active) {
				setLocalRotation(getLocalRotation().mult(rotationAmountLeft));
			}
		} else {
			final Vector3f vector = active ? TURN_LEFT : TURN_RIGHT;
			walkDirection.addLocal(vector);
		}
		setWalkDirection(walkDirection);
	}

	/**
	 * Moves the spatial to the right of the map (default rotation mode) or
	 * rotate the spatial to the right (rotationEnabled mode).
	 * 
	 * @param active
	 *            is the action key pressed?
	 */
	public void turnRight(boolean active) {
		if (rotationEnabled) {
			if (active) {
				setLocalRotation(getLocalRotation().mult(rotationAmountRight));
			}
		} else {
			final Vector3f vector = active ? TURN_RIGHT : TURN_LEFT;
			walkDirection.addLocal(vector);
		}
		setWalkDirection(walkDirection);

	}

	/**
	 * Moves the spatial to the top of the map (default rotation mode) or move
	 * the spatial to the spatial's look direction (rotationEnabled mode).
	 * 
	 * @param active
	 *            is the action key pressed?
	 */
	public void moveForward(boolean active) {
		final Vector3f vector = active ? MOVE_FORWARD : MOVE_BACKWARD;
		walkDirection.addLocal(vector);
		setWalkDirection(walkDirection);
	}

	/**
	 * Moves the spatial to the bottom of the map (default rotation mode) or
	 * move the spatial to the inverse spatial's look direction (rotationEnabled
	 * mode).
	 * 
	 * @param active
	 *            is the action key pressed?
	 */
	public void moveBackward(boolean active) {
		final Vector3f vector = active ? MOVE_BACKWARD : MOVE_FORWARD;
		walkDirection.addLocal(vector);
		setWalkDirection(walkDirection);
	}

	@Override
	public void setWalkDirection(Vector3f vec) {
		if (rotationEnabled) {
			super.setWalkDirection(getLocalRotation().mult(vec, walkDirectionAux).negate());
		} else {
			super.setWalkDirection(vec);
		}
		if (!rotationEnabled && !vec.equals(Vector3f.ZERO)) {
			int direction = 1;
			if (vec.x < 0) {
				direction = -1;
			}
			setLocalRotation(rotationAux.fromAngles(0, vec.normalize().angleBetween(Vector3f.UNIT_Z)
					* direction, 0));
		}
	}

	/**
	 * Change the default rotation mode.
	 * 
	 * @param rotationEnabled
	 *            <code>true</code> enable de spatial rotation mode.
	 */
	public void setRotationEnabled(boolean enableRotation) {
		this.rotationEnabled = enableRotation;
		if (enableRotation) {
			setRotationSpeed(rotationSpeed);
		}
	}

	/**
	 * When the spatial rotation mode is enabled (
	 * <code>setRotationEnabled(true)</code>) the turn amount is determined by
	 * this factor. By default, de turn amount is 0.1 radians.
	 * 
	 * @param rotationSpeed
	 *            the new amount for the turns (in radians).
	 */
	public void setRotationSpeed(float rotationSpeed) {
		this.rotationSpeed = rotationSpeed;
		
		if (rotationAmountLeft == null) {
			rotationAmountLeft = new Quaternion().fromAngleNormalAxis(rotationSpeed, Vector3f.UNIT_Y);
		} else {
			rotationAmountLeft.fromAngleNormalAxis(rotationSpeed, Vector3f.UNIT_Y);
		}
		
		if (rotationAmountRight == null) {
			rotationAmountRight = new Quaternion().fromAngleNormalAxis(-rotationSpeed, Vector3f.UNIT_Y);
		} else {
			rotationAmountRight.fromAngleNormalAxis(-rotationSpeed, Vector3f.UNIT_Y);
		}
		
		if (walkDirectionAux == null) {
			walkDirectionAux = new Vector3f();
		}
	}
}