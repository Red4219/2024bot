package frc.robot.subsystems;

import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants.kArmPoses;
import frc.robot.Constants.Mode;
import frc.robot.Tools.PhotonVision;

public class ArmSubsystem extends SubsystemBase {

	// region properties

	public HashMap<kArmPoses, double[]> armStates = ArmConstants.kArmStatesMap;

	/** used to track the state of the arm */
	private kArmPoses targetArmState;

	/** controls the side of the robot the arm is on */
	//private boolean isFront;
	private boolean enableArm;
	private boolean autoAim = false;

	// endregion

	// new
	private CANSparkMax rightMotor, leftMotor;
	public SparkPIDController rightPIDController;
	private PIDController pidController;
	private RelativeEncoder rightEncoder, leftEncoder;
	private AbsoluteEncoder rightBoreEncoder;

	private double position;
	private boolean atSetPoint = false;
	private double pidOutput;
	private double targetPosition = 0.0;

	private PhotonVision _photonVision;
	private int speakerTarget = 0;
	//

	public ArmSubsystem(PhotonVision photonVision) {
		
		rightMotor = new CANSparkMax(frc.robot.Constants.ArmConstants.kRightArmPort, MotorType.kBrushless);
		leftMotor = new CANSparkMax(frc.robot.Constants.ArmConstants.kLeftArmPort, MotorType.kBrushless);

		if(Constants.getMode() == Mode.SIM) {
			REVPhysicsSim.getInstance().addSparkMax(rightMotor, 2.6f, 5676);
			REVPhysicsSim.getInstance().addSparkMax(leftMotor, 2.6f, 5676);
		}

		rightMotor.restoreFactoryDefaults();
		leftMotor.restoreFactoryDefaults();

		rightMotor.setInverted(true);
		//rightMotor.setSmartCurrentLimit(Constants.ArmConstants.kArmCurrentLimit);
		//rightMotor.setSecondaryCurrentLimit(Constants.ArmConstants.kArmCurrentLimit + 3);

		rightMotor.setIdleMode(IdleMode.kBrake);
		leftMotor.setIdleMode(IdleMode.kBrake);

		// Have the left follow the right inverted
		leftMotor.follow(rightMotor, true);

		rightEncoder = rightMotor.getEncoder();
		rightBoreEncoder = rightMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
		leftEncoder = leftMotor.getEncoder();

		rightPIDController = rightMotor.getPIDController();
		rightPIDController.setFeedbackDevice(rightBoreEncoder);

		rightPIDController.setOutputRange(-Constants.ArmConstants.kArmCurrentLimit, Constants.ArmConstants.kArmCurrentLimit);

		this.rightPIDController.setP(Constants.ArmConstants.kArmGains.kP);
		this.rightPIDController.setI(Constants.ArmConstants.kArmGains.kI);
		this.rightPIDController.setD(Constants.ArmConstants.kArmGains.kD);

		// tells the pid controller on the arms to use trapezoidal constraints 
		rightPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);
		rightPIDController.setSmartMotionAllowedClosedLoopError(0, 0);

		pidController = new PIDController(Constants.ArmConstants.kArmGains.kP, Constants.ArmConstants.kArmGains.kI, Constants.ArmConstants.kArmGains.kD);
		pidController.setTolerance(Constants.ArmConstants.kTolerance, 10);

		if(Constants.debugArm == true) {

			ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
			armTab.addDouble("Arm Position", this::getPosition);
			armTab.addBoolean("Auto Aim", this::getAutoAim);
			armTab.addBoolean("At Set Point", this::atSetPoint);
			armTab.addDouble("Target Position", this::getTargetPosition);
			armTab.addDouble("Right Applied Output", this::getRightAppliedOutput);
			armTab.addDouble("Left Applied Output", this::getLeftAppliedOutput);
			armTab.addDouble("Right Output Current", this::getRightOutputCurrent);
			armTab.addDouble("Left Output Current", this::getLeftOutputCurrent);
			armTab.addDouble("Right Temp", this::getRightTemp);
			armTab.addDouble("Left Temp", this::getLeftTemp);
			armTab.addString("State Name", this::getTargetStateName);
			armTab.addDouble("Voltage", this::getVoltage);
		}

		_photonVision = photonVision;

		if(DriverStation.getAlliance().isPresent()) {
            if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                speakerTarget = 7;
            } else {
                speakerTarget = 4;
            }
        }

		this.targetArmState = kArmPoses.SPEAKER_SCORE_POST;
		this.targetPosition = armStates.get(targetArmState)[0];
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		if(Constants.enableLogger == true) {

			Logger.recordOutput("Arm/position", rightEncoder.getPosition());
			Logger.recordOutput("Arm/target", targetPosition);
			Logger.recordOutput("Arm/autoAim", autoAim);
			Logger.recordOutput("Arm/rightAppliedOutput", rightMotor.getAppliedOutput());
			Logger.recordOutput("Arm/leftAppliedOutput", leftMotor.getAppliedOutput());
			Logger.recordOutput("Arm/rightOutputCurrent", rightMotor.getOutputCurrent());
			Logger.recordOutput("Arm/leftOutputCurrent", leftMotor.getOutputCurrent());
			Logger.recordOutput("Arm/rightMotorTemp", rightMotor.getMotorTemperature());
			Logger.recordOutput("Arm/leftMotorTemp", leftMotor.getMotorTemperature());
		}

		if(speakerTarget == 0) {
			if(DriverStation.getAlliance().isPresent()) {
            	if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                	speakerTarget = 7;
            	} else {
                	speakerTarget = 4;
            	}
        	}
		}


		if(targetArmState == kArmPoses.AIM) {
			if(Constants.kEnablePhotonVision) {
				targetPosition = calculatePosition(_photonVision.targetDistance(speakerTarget));
			}
		}

		if(Constants.getMode() == Mode.REAL) {
			position = this.rightBoreEncoder.getPosition();

			if(Math.abs(rightBoreEncoder.getPosition() - targetPosition) <= Constants.ArmConstants.kTolerance) {
				atSetPoint = true;
				//System.out.println("setting atSetPoint to true");
			} else {
				atSetPoint = false;
				//System.out.println("setting atSetPoint to false");
			}
		} else if(Constants.getMode() == Mode.SIM) {
			position = this.rightMotor.getEncoder().getPosition();
		}
		

		setReference();


		//updateSequencing();
	}

	// region Commands

	/*public Command UnsequencedArmPoseCommand(final kArmPoses state) {
		return runOnce(() -> {
			setUnsequencedArmState(state);
		});
	}*/

	/*public Command SequencedArmPoseCommand(final kArmPoses state) {
		return runOnce(() -> {
			setSequencedArmState(state);
		});
	}*/

	/** Toggles the dominant side of the robot */
	/*public void ToggleSide() {
		isFront = !isFront;
		majorArm.setSign((isFront) ? 1 : -1);
		minorArm.setSign((isFront) ? 1 : -1);
		majorArm.setReference();
		minorArm.setReference();
	}*/

	/*public Command toggleArmMotors() {
		return runOnce(() -> {
			enableArms = false;
			minorArm.toggleMotors();
			majorArm.toggleMotors();
		});
	}*/

	public Command zeroArms() {
		return runOnce(() -> {
			//minorArm.resetZeros();
			//majorArm.resetZeros();
		});
	}



	public void setSequencedArmState(kArmPoses state) {
		setTargetArmState(state);
	}

	/**
	 * @param targetArmState the targetArmState to set
	 */
	public void setTargetArmState(kArmPoses state) {

		/*if(this.targetArmState == state) {
			// we are calling for the state we are already in
			return;
		}*/

		targetArmState = state;
		enableArm = true;

		// get minor speed from map
		// gets the angle values from the hashmap
		//majorArm.setTargetTheta(armStates.get(targetArmState)[0]);

		if(state != kArmPoses.AIM && state != kArmPoses.IDLE) {
			targetPosition = armStates.get(targetArmState)[0];
		}

		atSetPoint = false;
	}

	public void updateSequencing() {
		/*if ((majorArm.getAtTarget(30) || minorArm.getAtTarget(30)) && enableArms) {
			majorArm.setReference();
			minorArm.setReference();
		}*/
	}

	public void moveToPosition(double position) {
		//System.out.println("ArmSubSystem::moveToPosition() - position: " + position);

		if(position < Constants.ArmConstants.kMinHeight) {
			position = Constants.ArmConstants.kMinHeight;
		} else if(position > Constants.ArmConstants.kMaxHeight) {
			position = Constants.ArmConstants.kMaxHeight;
		}

		targetPosition = position;
	}

	public void setReference() {
		//if(atSetPoint == false) {
			//System.out.println("atSetPoint is false");
			if(Constants.getMode() == Mode.REAL) {
				//System.out.println("it is being called, targetPosition: " + targetPosition);
				//REVLibError error = rightPIDController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);
				//REVLibError error = rightPIDController.setReference(targetPosition, CANSparkMax.ControlType.kSmartMotion);

				/*if(error != REVLibError.kOk) {
					System.out.println("ClimberSubsystem::setReference() - Error - could not set the PID controller");
				}*/

				// Set the min/max values for the height of the arm
				if(targetPosition < Constants.ArmConstants.kMinHeight) {
					targetPosition = Constants.ArmConstants.kMinHeight;
				} else if(targetPosition > Constants.ArmConstants.kMaxHeight) {
					targetPosition = Constants.ArmConstants.kMaxHeight;
				}

				pidOutput = pidController.calculate(rightBoreEncoder.getPosition(), targetPosition);

				if(pidOutput > 12) {
					pidOutput = 12;
				} else if(pidOutput < -12) {
					pidOutput = -12;
				}

				rightMotor.setVoltage(pidOutput);

				//System.out.println("pidOutput: " + pidOutput);

				// Are we close to the target position?
				/*if(Math.abs(rightBoreEncoder.getPosition() - targetPosition) <= Constants.ArmConstants.kTolerance) {
					// Yes we are
					atSetPoint = true;
					//System.out.println("setting atSetPoint to true");
				}*/
			} else if(Constants.getMode() == Mode.SIM) {
				// We are not at the set point and are in SIM

				/*pidOutput = pidController.calculate(rightEncoder.getPosition(), targetPosition);

				if(pidOutput > 12) {
					pidOutput = 12;
				} else if(pidOutput < -12) {
					pidOutput = -12;
				}

				if(pidController.atSetpoint()) {
					System.out.println("we are at the setpoint, position: " + rightEncoder.getPosition());
					atSetPoint = true;
					//if(Constants.getMode() == Mode.SIM) {
					//	rightMotor.setVoltage(0.0);
					//}
				} else {
					// we have not reached the set point yet
					rightMotor.setVoltage(pidOutput);
					//System.out.println("adjusting position: " + rightEncoder.getPosition());
				}*/

				// Commented out the above code because in the simulator, it never gets to the proper position
				position = targetPosition;
				this.atSetPoint = true;
			}

			

			
		/* } else {
			//System.out.println("we are at the setpoint, position: " + rightEncoder.getPosition());

			if(Constants.getMode() == Mode.SIM) {
				rightMotor.setVoltage(0.0);
			}
		}*/

		/*if(Math.abs(rightBoreEncoder.getPosition() - targetPosition) <= Constants.ArmConstants.kTolerance) {
			// Yes we are
			//atSetPoint = true;
			//rightMotor.setVoltage(0.0);
		} else {
			//atSetPoint = false;
		}*/
	}

	public double getPosition() {
		return position;
	}

	public double getTargetPosition() {
		return targetPosition;
	}

	public boolean getAutoAim() {
		
		if(targetArmState == kArmPoses.AIM) {
			return true;
		}

		return false;
	}

	// endregion

	// region getters

	/**
	 * Used to get the target height of the arm as an enum
	 * 
	 * @return armState: can be (LOW_SCORE, MID_SCORE, HIGH_SCORE, LOW_INTAKE,
	 *         MID_INTAKE, HIGH_INTAKE)
	 */
	public kArmPoses getArmState() {
		return targetArmState;
	}

	/** ruturns true if the target dominant side of the robot is front */
	/*public boolean getIsFront() {
		return isFront;
	}*/

	public boolean isAtSetPoint() {
		return atSetPoint;
	}

	public double getRightAppliedOutput() {
		return rightMotor.getAppliedOutput();
	}

	public double getLeftAppliedOutput() {
		return leftMotor.getAppliedOutput();
	}

	public double getRightTemp() {
		// Convert from celcius to Fahrenheit
		return (rightMotor.getMotorTemperature() * 9 / 5) + 32;
	}

	public double getLeftTemp() {
		// Convert from celcius to Fahrenheit
		return (leftMotor.getMotorTemperature() * 9 / 5) + 32;
	}

	// motor controller's output current in Amps
	public double getRightOutputCurrent() {
		return rightMotor.getOutputCurrent();
	}

	// motor controller's output current in Amps
	public double getLeftOutputCurrent() {
		return leftMotor.getOutputCurrent();
	}

	public boolean atSetPoint() {
		return this.atSetPoint;
	}

	public String getTargetStateName() {
		String stateName;
		switch(targetArmState) {
			case AIM:
				stateName = "AIM";
				break;
			case AMP_SCORE:
				stateName = "AMP_SCORE";
				break;
			case GROUND_INTAKE:
				stateName = "GROUND_INTAKE";
				break;
			case HUMAN_ELEMENT_INTAKE:
				stateName = "HUMAN_ELEMENT_INTAKE";
				break;
			case IDLE:
				stateName = "IDLE";
				break;
			case SPEAKER_SCORE:
				stateName = "SPEAKER_SCORE";
				break;
			case TRAP_DOOR_SCORE:
				stateName = "TRAP_DOOR_SCORE";
				break;
			default:
				stateName = "UNKNOWN";
				break;
		}

		return stateName;
	}

	double getVoltage() {
		return this.pidOutput;
	}

	double calculatePosition(double distance) {
		return distance;
	}

	/*public boolean getAtTarget(double deadBand) {

		if(getPosition() == armStates.get(targetArmState)[0]) {
			//System.out.println("they match");
			return true;
		} 

		//System.out.println("they do not match position: " + getPosition() + " target: " + armStates.get(targetArmState)[0]);
		
		return false;
	}*/

	/*public double[] getTargetTheta() {
		return new double[] { majorArm.getTargetTheta() };
	}*/

	// endregion

}
