// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants.kArmPoses;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.kIntakeStates;
import frc.robot.Mechanisms.ColorSensor;
import frc.robot.Mechanisms.IntakeWheels;
import frc.robot.Mechanisms.Vaccum;
import frc.robot.Tools.Parts.IRDistanceSensor;

public class IntakeSubsystem extends SubsystemBase {

	//private Vaccum centerSucker;

	//private PneumaticHub pneumaticHub;
	//private Solenoid centerDumpSolenoid;
	//private Solenoid centerSealerSolenoid;

	private IntakeWheels intake = new IntakeWheels(Constants.IntakeConstants.kIntakeWheelPort);
	private ColorSensor colorSensor = new ColorSensor();

	private kIntakeStates currentIntakeState;

	//private IRDistanceSensor distanceSensor;

	/** Creates a new IntakeSubsystem. */
	public IntakeSubsystem() {

		//pneumaticHub = new PneumaticHub(IntakeConstants.kPnemnaticHubPort);

		//centerDumpSolenoid = pneumaticHub.makeSolenoid(IntakeConstants.kCenterDumpSolenoidPort);
		//centerSealerSolenoid = pneumaticHub.makeSolenoid(IntakeConstants.kCenterSealerSolenoidPort);

		//distanceSensor = new IRDistanceSensor(0);

		currentIntakeState = kIntakeStates.DISABLED;

		/*centerSucker = new Vaccum(
				IntakeConstants.kCenterSuckerPort,
				IntakeConstants.kCenterSuckerCurrentLimit,
				true,
				centerDumpSolenoid,
				centerSealerSolenoid);*/


		//gripper = new IntakeWheels(IntakeConstants.kRightIntakeWheelPort, IntakeConstants.kLeftIntakeWheelPort);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		//SmartDashboard.putNumber("Center Sucker RPM", centerSucker.getRPM());
		//SmartDashboard.putNumber("Center Sucker Current Draw", centerSucker.getMotorCurrentDraw());
		//SmartDashboard.putBoolean("Has CUBE", getHasCube());
		//SmartDashboard.putBoolean("Has CONE", getHasCone()); 
		//SmartDashboard.putNumber("distance sensor", distanceSensor.getDistanceAsVolts());
	}

	// region commands

	public InstantCommand intakeCommand() {
		return new InstantCommand(() -> setIntakeState(kIntakeStates.INTAKE));
	}

	public InstantCommand outtakeCommand() {
		return new InstantCommand(() -> setIntakeState(kIntakeStates.OUTTAKE));
	}

	public InstantCommand idleCommand() {
		return new InstantCommand(() -> setIntakeState(kIntakeStates.IDLE));
	}

	public InstantCommand disableCommand() {
		return new InstantCommand(() -> setIntakeState(kIntakeStates.DISABLED));
	}

	/*public void intakeTime(long timeToIntake) {
		TimerTask task = new TimerTask() {
            public void run() {
                setIntakeState(kIntakeStates.IDLE);
                System.out.println("stopping the intake");
            }
        };
        Timer timer = new Timer("Timer");
    
        timer.schedule(task, timeToIntake);

        setIntakeState(kIntakeStates.INTAKE);
        System.out.println("starting the intake");
	}*/

	/*public InstantCommand timeIntakeCommand(long timeToIntake) {
		return new InstantCommand(() -> intakeTime(timeToIntake));
	}*/

	public boolean noteDetected() {
		return false;
	}

	public void intakeNote() {
		intake.Intake();
	}

	public void outtakeNote() {
		intake.OutTake();
	}

	public void disableIntake() {
		intake.disable();
	}

	public void setIntakeState(kIntakeStates state) {

		currentIntakeState = state;

		switch (currentIntakeState) {

			case IDLE:
				disableIntake();
				break;

			case INTAKE:
				intakeNote();
				break;

			case OUTTAKE:
				outtakeNote();
				break;

			case DISABLED:
				disableIntake();
				break;

		}
	}	

	public void updateIntakeFromArmPose(kArmPoses armPose) {
		setIntakeState(IntakeConstants.kArmStateToIntakeStateMap.get(armPose));
	}

	public boolean hasNote() {
		// finish this
		return false;
	}

	/*public boolean getHasCone() {
		if (centerSucker.getRPM() < IntakeConstants.kHasConeThreshold && centerSucker.getRPM() > 10) {
			return true;
		}
		return false;
	}*/

	/*public double getDistanceAsVolts() {
		return distanceSensor.getDistanceAsVolts();
	}*/
}
