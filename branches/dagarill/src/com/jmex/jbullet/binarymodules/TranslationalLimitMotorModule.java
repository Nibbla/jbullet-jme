package com.jmex.jbullet.binarymodules;

import java.io.IOException;

import com.bulletphysics.dynamics.constraintsolver.TranslationalLimitMotor;
import com.jme.util.export.InputCapsule;
import com.jme.util.export.Savable;
import com.jme.util.export.binary.BinaryLoaderModule;

public class TranslationalLimitMotorModule implements BinaryLoaderModule  {

	
	private TranslationalLimitMotor motor;
	
	public TranslationalLimitMotorModule(TranslationalLimitMotor motor){
		this.motor=motor;
	}

	@Override
	public String getKey() {
		
		return com.jmex.jbullet.joints.motors.TranslationalLimitMotor.class.getName();
	}

	@Override
	public Savable load(InputCapsule arg0) throws IOException {
		return new com.jmex.jbullet.joints.motors.TranslationalLimitMotor(motor);
	}

	

}
