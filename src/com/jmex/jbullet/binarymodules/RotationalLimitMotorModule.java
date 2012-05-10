package com.jmex.jbullet.binarymodules;

import java.io.IOException;

import com.bulletphysics.dynamics.constraintsolver.RotationalLimitMotor;
import com.bulletphysics.dynamics.constraintsolver.TranslationalLimitMotor;
import com.jme.util.export.InputCapsule;
import com.jme.util.export.Savable;
import com.jme.util.export.binary.BinaryLoaderModule;

public class RotationalLimitMotorModule implements BinaryLoaderModule  {

	
	private RotationalLimitMotor motor;
	
	public RotationalLimitMotorModule(RotationalLimitMotor motor){
		this.motor=motor;
	}

	@Override
	public String getKey() {
		
		return com.jmex.jbullet.joints.motors.RotationalLimitMotor.class.getName();
	}

	@Override
	public Savable load(InputCapsule arg0) throws IOException {
		return new com.jmex.jbullet.joints.motors.RotationalLimitMotor(motor);
	}

	

}
