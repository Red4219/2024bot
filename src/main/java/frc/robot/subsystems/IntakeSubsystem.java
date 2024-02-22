// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Timer;
import java.util.TimerTask;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants.kArmPoses;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.IntakeConstants.kIntakeStates;
import frc.robot.Mechanisms.ColorSensor;
import frc.robot.Mechanisms.IntakeWheels;


public class IntakeSubsystem extends SubsystemBase {

	//private Vaccum centerSucker;

	//private PneumaticHub pneumaticHub;
	//private Solenoid centerDumpSolenoid;
	//private Solenoid centerSealerSolenoid;

	private IntakeWheels intakeWheels = new IntakeWheels(Constants.IntakeConstants.kIntakeWheelPort);
	private ColorSensor colorSensor = new ColorSensor();
	private boolean hasNote = false;

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

		ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
		intakeTab.addDouble("Intake Current", intakeWheels::getOutputCurrent);
		intakeTab.addDouble("Intake Temp", intakeWheels::getTemp);
		intakeTab.addDouble("Intake Output", intakeWheels::getAppliedOutput);
		intakeTab.addBoolean("Note Detected", colorSensor::noteDetected);
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		//SmartDashboard.putNumber("Center Sucker RPM", centerSucker.getRPM());
		//SmartDashboard.putNumber("Center Sucker Current Draw", centerSucker.getMotorCurrentDraw());
		//SmartDashboard.putBoolean("Has CUBE", getHasCube());
		//SmartDashboard.putBoolean("Has CONE", getHasCone()); 
		//SmartDashboard.putNumber("distance sensor", distanceSensor.getDistanceAsVolts());

		//colorSensor.printColorValues();

		//SmartDashboard.putBoolean("Has Note", colorSensor.noteDetected());
		Logger.recordOutput("Intake/Note_Detected", colorSensor.noteDetected());

		//intakeWheels.Intake();

		/*switch (currentIntakeState) {

			case IDLE:
				System.out.println("IntakeSubsystem::periodic() - IDLE state");
				break;

			case INTAKE:
				System.out.println("IntakeSubsystem::periodic() - INTAKE state");
				break;

			case OUTTAKE:
				System.out.println("IntakeSubsystem::periodic() - OUTTAKE state");
				break;

			case DISABLED:
				System.out.println("IntakeSubsystem::periodic() - DISABLED state");
				break;

			case BUMP:
				System.out.println("IntakeSubsystem::periodic() - BUMP state");
				break;
		}*/

		

		/*if(this.currentIntakeState == kIntakeStates.BUMP) {

		} else {
			if(colorSensor.noteDetected()) {
				hasNote = true;
				setIntakeState(kIntakeStates.IDLE);
			}
		}*/
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
		return colorSensor.noteDetected();
	}

	public void intakeNote() {
		intakeWheels.Intake();
	}

	public void outtakeNote() {
		intakeWheels.OutTake();
	}

	public void disableIntake() {
		intakeWheels.disable();
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

			case BUMP:
				intakeNote();
				break;

		}
	}	

	public void updateIntakeFromArmPose(kArmPoses armPose) {
		setIntakeState(IntakeConstants.kArmStateToIntakeStateMap.get(armPose));
	}

	public boolean hasNote() {
		return this.hasNote;
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
