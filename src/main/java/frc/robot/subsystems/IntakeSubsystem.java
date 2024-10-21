// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Timer;
import java.util.TimerTask;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.REVPhysicsSim;

import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
	//private ColorSensor colorSensor = new ColorSensor();
	private ColorSensor colorSensor;
	private boolean hasNote = false;
	private String stateName = "DISABLED";

	private kIntakeStates currentIntakeState;

	private CommandXboxController driverController;

	//private IRDistanceSensor distanceSensor;

	/** Creates a new IntakeSubsystem. */
	public IntakeSubsystem() {

		if (Constants.kEnableIntake) {

			if (Constants.IntakeConstants.kEnableColorSensor) {
				colorSensor = new ColorSensor();
			}

			currentIntakeState = kIntakeStates.DISABLED;

			if (Constants.debugIntake == true) {
				ShuffleboardTab intakeTab = Shuffleboard.getTab("Intake");
				intakeTab.addDouble("Intake Current", intakeWheels::getOutputCurrent);
				intakeTab.addDouble("Intake Temp", intakeWheels::getTemp);
				intakeTab.addDouble("Intake Output", intakeWheels::getAppliedOutput);
				intakeTab.addInteger("Blue", colorSensor::getBlue);
				intakeTab.addInteger("Red", colorSensor::getRed);
				intakeTab.addInteger("Green", colorSensor::getGreen);
				intakeTab.addString("State Name", this::getStateName);

				if (Constants.IntakeConstants.kEnableColorSensor) {
					intakeTab.addBoolean("Note Detected", colorSensor::noteDetected);
				}
			}
		}
	}

	public void setDriverController(CommandXboxController driverController) {
		this.driverController = driverController;
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

		if(Constants.IntakeConstants.kEnableNoteDetectedRumble == true) {
			if(colorSensor.noteDetected() == true) {
				this.driverController.getHID().setRumble(RumbleType.kLeftRumble, 1.0);
			} else {
				this.driverController.getHID().setRumble(RumbleType.kLeftRumble, 0.0);
			}
		}

		if(Constants.enableLogger == true) {
			if(Constants.IntakeConstants.kEnableColorSensor == true) {
				Logger.recordOutput("Intake/Note_Detected", colorSensor.noteDetected());
			}
		}

		/*if(Constants.IntakeConstants.kEnableColorSensor == true) {

			if(this.currentIntakeState == Constants.IntakeConstants.kIntakeStates.INTAKE) {
				if(colorSensor.noteDetected()) {
					this.hasNote = true;
					intakeWheels.disable();
				}
			}
		}*/

		switch(this.currentIntakeState) {
			case BUMP:
				this.stateName = "BUMP";
				break;
			case DISABLED:
				this.stateName = "DISABLED";
				break;
			case IDLE:
				this.stateName = "IDLE";
				break;
			case INTAKE:
				this.stateName = "INTAKE";
				if(Constants.IntakeConstants.kEnableColorSensor == true) {
					if(colorSensor.noteDetected()) {
						this.hasNote = true;
						intakeWheels.disable();
					}
				}
				break;
			case INTAKE_SLOW:
				this.stateName = "INTAKE_SLOW";
				if(Constants.IntakeConstants.kEnableColorSensor == true) {
					if(colorSensor.noteDetected()) {
						this.hasNote = true;
						intakeWheels.disable();
					}
				}
				break;
			case INTAKE_IGNORE_NOTE:
				this.stateName = "INTAKE_IGNORE_NOTE";
				break;
			case OUTTAKE:
				this.stateName = "OUTTAKE";
				break;
			default:
				break;
			
		}

		

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
		if(Constants.IntakeConstants.kEnableColorSensor == true) {
			return colorSensor.noteDetected();
		}

		return false;
	}

	public void intakeNote() {
		intakeWheels.Intake();
	}

	public void intakeNoteSlowly() {
		intakeWheels.IntakeSlowly();
	}

	public void intakeNoteIgnoreNote() {
		intakeWheels.Intake();
	}

	public void outtakeNote() {
		intakeWheels.OutTake();
	}

	public void disableIntake() {
		intakeWheels.disable();
	}

	public void setIntakeState(kIntakeStates state) {

		if(state == currentIntakeState) {
			disableIntake();
			currentIntakeState = kIntakeStates.DISABLED;
		} else {

			currentIntakeState = state;

			switch (currentIntakeState) {

				case IDLE:
					disableIntake();
					break;

				case INTAKE:
					intakeNote();
					break;
				
				case INTAKE_SLOW:
					intakeNoteSlowly();
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

				case INTAKE_IGNORE_NOTE:
					intakeNoteIgnoreNote();
					break;

			}
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

	public String getStateName() {
		return this.stateName;
	}
}
