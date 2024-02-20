// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.robot.Constants.Mode;;

/** Add your docs here. */
public class IntakeWheels {

	private CANSparkMax _intakeSparkMax;

	public IntakeWheels(int port) {
		_intakeSparkMax = new CANSparkMax(port, MotorType.kBrushless);

		if(Constants.getMode() == Mode.SIM) {
			REVPhysicsSim.getInstance().addSparkMax(_intakeSparkMax, 2.6f, 5676);
		}

		_intakeSparkMax.clearFaults();
		_intakeSparkMax.restoreFactoryDefaults();
		_intakeSparkMax.setIdleMode(IdleMode.kBrake);
		//_intakeSparkMax.setSmartCurrentLimit(Constants.IntakeConstants.kSmartCurrentLimit);
	}

	public void Intake() {
		//System.out.println("IntakeWheels::Intake() called");

		if(Constants.getMode() == Mode.SIM) {
			_intakeSparkMax.setVoltage(1.0);
		} else {
			//System.out.println("Intake() being called");
			_intakeSparkMax.set(Constants.IntakeConstants.kIntakeSpeed);
		}
	}

	public void OutTake() {
		//System.out.println("IntakeWheels::OutTake() called");
		
		/*if(Constants.getMode() == Mode.SIM) {
			_intakeSparkMax.setVoltage(1.0);
		} else {
			_intakeSparkMax.set(Constants.IntakeConstants.kOuttakeSpeed);
		}*/
	}

	public void disable() {
		//System.out.println("IntakeWheels::disable() called");

		/*if(Constants.getMode() == Mode.SIM) {
			_intakeSparkMax.setVoltage(0.0);
		} else {
			_intakeSparkMax.stopMotor();
		}*/
	}

	public double getOutputCurrent() {
		return _intakeSparkMax.getOutputCurrent();
	}

	public double getTemp() {
		return (_intakeSparkMax.getMotorTemperature() * 9 / 5) + 32;
	}

	public double getAppliedOutput() {
		return _intakeSparkMax.getAppliedOutput();
	}

	public boolean hasNote() {
		if(getOutputCurrent() > Constants.IntakeConstants.kIntakeOutputCurrentThreshold) {
			return true;
		}

		return false;
	}
}
