package frc.robot.subsystems;

import java.util.EnumSet;
import java.util.HashMap;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController.AccelStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ClimberConstants.kClimberPoses;
import frc.robot.Tools.JoystickUtils;

public class ClimberSubsystem extends SubsystemBase {

	/** used to track the state of the climber */
	private kClimberPoses targetClimberState;
	private boolean enableClimber;

	private CANSparkMax rightMotor, leftMotor;
	public SparkPIDController rightPIDController, leftPIDController;
	private RelativeEncoder rightEncoder, leftEncoder;

	private ProfiledPIDController profiledPIDController;
	private PIDController pidControllerRight;
	private PIDController pidControllerLeft;

	private boolean invertLeader = true;

	private boolean atSetPointRight = true;
	private boolean atSetPointLeft = true;
	private double targetPositionLeft = 0.0;
	private double targetPositionRight = 0.0;
	double pidOutputLeft = 0.0;
	double pidOutputRight = 0.0;
	private CommandXboxController operatorController;

	HashMap<kClimberPoses, double[]> climberStates = Constants.ClimberConstants.kClimberStatesMap;

	private static ShuffleboardTab ClimberTab = Shuffleboard.getTab("Climber");
	//private static GenericEntry ClimberPosition = ClimberTab.addPersistent("Climber Position", 0).getEntry();
	//private static GenericEntry ClimberTarget = ClimberTab.addPersistent("Climber Target", 0).getEntry();

	//private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
	//private static GenericEntry climberNTPosition = ClimberTab.addPersistent("Climber NT Value", 0).getEntry();

	//NetworkTable climberTable = networkTableInstance.getTable("/climber");
	//NetworkTableEntry entry;

	public ClimberSubsystem(CommandXboxController operatorController) {
		rightMotor = new CANSparkMax(frc.robot.Constants.ClimberConstants.kRightPort, MotorType.kBrushless);
		leftMotor = new CANSparkMax(frc.robot.Constants.ClimberConstants.kLeftPort, MotorType.kBrushless);

		this.operatorController = operatorController;

		if(Constants.getMode() == Mode.SIM) {
			REVPhysicsSim.getInstance().addSparkMax(rightMotor, 2.6f, 5676);
			REVPhysicsSim.getInstance().addSparkMax(leftMotor, 2.6f, 5676);
		}

		// Setup the right climber motor
		if(rightMotor != null) {
			rightMotor.restoreFactoryDefaults();
			rightMotor.setIdleMode(IdleMode.kBrake);
			rightEncoder = rightMotor.getEncoder();

			// Tells the motors to automatically convert degrees to rotations
			/*if(Constants.getMode() == Mode.REAL) {
				rightEncoder.setPositionConversionFactor((2 * Math.PI) / frc.robot.Constants.ClimberConstants.kClimberTicks);
				rightEncoder.setVelocityConversionFactor((2 * Math.PI) / frc.robot.Constants.ClimberConstants.kClimberTicks);
			} else {
				rightEncoder.setPositionConversionFactor((2 * Math.PI) / frc.robot.Constants.ArmConstants.kMajorArmTicks);
				rightEncoder.setVelocityConversionFactor((2 * Math.PI) / frc.robot.Constants.ArmConstants.kMajorArmTicks);
			}*/

			rightEncoder.setPosition(0);

			rightPIDController = rightMotor.getPIDController();
			rightPIDController.setFeedbackDevice(rightMotor.getEncoder());

			// tells the pid controller on the arms to use trapezoidal constraints 
			rightPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

			rightPIDController.setSmartMotionAllowedClosedLoopError(0, 0);

			rightPIDController.setP(Constants.ClimberConstants.kClimberGains.kP);
			rightPIDController.setI(Constants.ClimberConstants.kClimberGains.kI);
			rightPIDController.setD(Constants.ClimberConstants.kClimberGains.kD);

			rightPIDController.setIZone(0);
			rightPIDController.setFF(0);

			rightPIDController.setSmartMotionMaxVelocity(Constants.ArmConstants.kMaxVelRadiansPerSec, 0);
			rightPIDController.setSmartMotionMinOutputVelocity(0, 0);
			//rightPIDController.setSmartMotionMaxAccel(Constants.ArmConstants. maxAccel, 0);

			rightPIDController.setOutputRange(-Constants.ArmConstants.kPIDOutputLimit, Constants.ArmConstants.kPIDOutputLimit);

			//pidControllerRight = new PIDController(1,.5,.5);
			pidControllerRight = new PIDController(Constants.ClimberConstants.kClimberGains.kP,Constants.ClimberConstants.kClimberGains.kI,Constants.ClimberConstants.kClimberGains.kD);
			pidControllerRight.setTolerance(Constants.ClimberConstants.kTolerance, 10);

			ClimberTab.addDouble("Target Right", this::getTargetPositionRight);
			ClimberTab.addDouble("Position Right", this::getPositionRight);
			ClimberTab.addDouble("Current Right", this::getRightCurrent);
			ClimberTab.addDouble("Temp Right", this::getRightTemp);
			ClimberTab.addDouble("Output Right", this::getRightOutput);
		}

		if(leftMotor != null) {
			leftMotor.restoreFactoryDefaults();
			leftMotor.setIdleMode(IdleMode.kBrake);
			leftEncoder = leftMotor.getEncoder();

			// Tells the motors to automatically convert degrees to rotations
			/*if(Constants.getMode() == Mode.REAL) {
				leftEncoder.setPositionConversionFactor((2 * Math.PI) / frc.robot.Constants.ClimberConstants.kClimberTicks);
				leftEncoder.setVelocityConversionFactor((2 * Math.PI) / frc.robot.Constants.ClimberConstants.kClimberTicks);
			} else {
				leftEncoder.setPositionConversionFactor((2 * Math.PI) / frc.robot.Constants.ArmConstants.kMajorArmTicks);
				leftEncoder.setVelocityConversionFactor((2 * Math.PI) / frc.robot.Constants.ArmConstants.kMajorArmTicks);
			}*/

			leftEncoder.setPosition(0);

			leftPIDController = leftMotor.getPIDController();
			leftPIDController.setFeedbackDevice(leftMotor.getEncoder());

			// tells the pid controller on the arms to use trapezoidal constraints 
			leftPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

			leftPIDController.setSmartMotionAllowedClosedLoopError(0, 0);

			leftPIDController.setP(Constants.ClimberConstants.kClimberGains.kP);
			leftPIDController.setI(Constants.ClimberConstants.kClimberGains.kI);
			leftPIDController.setD(Constants.ClimberConstants.kClimberGains.kD);

			leftPIDController.setIZone(0);
			leftPIDController.setFF(0);

			leftPIDController.setSmartMotionMaxVelocity(Constants.ArmConstants.kMaxVelRadiansPerSec, 0);
			leftPIDController.setSmartMotionMinOutputVelocity(0, 0);
			//leftPIDController.setSmartMotionMaxAccel(Constants.ArmConstants. maxAccel, 0);

			leftPIDController.setOutputRange(-Constants.ArmConstants.kPIDOutputLimit, Constants.ArmConstants.kPIDOutputLimit);

			//pidControllerLeft = new PIDController(1,.5,.5);
			pidControllerLeft = new PIDController(Constants.ClimberConstants.kClimberGains.kP,Constants.ClimberConstants.kClimberGains.kI,Constants.ClimberConstants.kClimberGains.kD);
			pidControllerLeft.setTolerance(Constants.ClimberConstants.kTolerance, 10);

			ClimberTab.addDouble("Target Left", this::getTargetPositionLeft);
			ClimberTab.addDouble("Position Left", this::getPositionLeft);
			ClimberTab.addDouble("Current Left", this::getLeftCurrent);
			ClimberTab.addDouble("Temp Left", this::getLeftTemp);
			ClimberTab.addDouble("Output Left", this::getLeftOutput);
		}

		//resetZeros();

		
		


		//setMaxOutput(frc.robot.Constants.ClimberConstants.kClimberCurrentLimit);

		

		

		

		

		//rightMotor.setSmartCurrentLimit(frc.robot.Constants.ClimberConstants.kClimberCurrentLimit);
		//rightMotor.setSecondaryCurrentLimit(Constants.ArmConstants.kArmCurrentLimit + 3);

		

		

		
		

		//////

		/*profiledPIDController = new ProfiledPIDController(
			0,
			1,
			0,
			new TrapezoidProfile.Constraints( // radians/s?
					2 * Math.PI * 600, // theoretical is 5676 RPM -> 94*2pi
					2 * Math.PI * 1200
			)
		);*/

		//if(Constants.getMode() == Mode.SIM) {

			//pidController = new PIDController(1,.5,.5);
			
			
			// Sets the error tolerance to 5, and the error derivative tolerance to 10 per second
			
			
		//}

		
		// Listen for changes
		//DoubleSubscriber positionSubscriber = climberTable.getDoubleTopic("position").subscribe(0.0);

		/*NetworkTableInstance.getDefault().addListener(
			positionSubscriber,
			EnumSet.of(NetworkTableEvent.Kind.kValueAll), 
			event -> {
				System.out.println("the value changed " + event.valueData.value.getDouble());

				//targetPosition = event.valueData.value.getDouble();

				//atSetPoint = false;
			}
		);*/

		

		
	}

	/*public boolean isAtSetPoint() {
		return atSetPoint;
	}*/

	public boolean isAtSetPointRight() {
		return atSetPointRight;
	}

	public boolean isAtSetPointLeft() {
		return atSetPointLeft;
	}

	/**
	 * sets the maximum output of the pid controller
	 * 
	 * @param maxOutput the max value that can be sent to the motor (0 - 1)
	 */
	/*public void setMaxOutput(double maxOutput) {
		rightPIDController.setOutputRange(-maxOutput, maxOutput);
	}*/

	/*public void setSmartCurrentLimit(int limit) {
		//rightMotor.setSmartCurrentLimit(limit);
		//leftMotor.setSmartCurrentLimit(limit);
		//rightMotor.setSecondaryCurrentLimit(limit + 3);
		//leftMotor.setSecondaryCurrentLimit(limit + 3);
	}*/

	/** used to disable the motors for rezeroing */
	//public void toggleMotors() {
		/*isRunning = !isRunning;

		if (isRunning) {
			rightMotor.stopMotor();
			leftMotor.stopMotor();
		} else {
			setReference();
		}*/
	//}

	/** Sets the pid referance point to the target theta of the segment */
	public void setReference() {

		// right
	
		if(atSetPointRight == false) {

			if(Constants.getMode() == Mode.REAL) {

				//System.out.println("it is being called, targetPosition: " + targetPosition);
				REVLibError error = rightPIDController.setReference(targetPositionRight, CANSparkMax.ControlType.kPosition);				

				if(error != REVLibError.kOk) {
					System.out.println("ClimberSubsystem::setReference() - Error - could not set the PID controller");
				}

				error = rightPIDController.setReference(targetPositionRight, CANSparkMax.ControlType.kPosition);

				// Are we close to the target position?
				if(Math.abs(rightEncoder.getPosition() - targetPositionRight) <= Constants.ClimberConstants.kTolerance) {
					// Yes we are
					atSetPointRight = true;
					System.out.println("setting atSetPoint to true");
				}

				if(Math.abs(leftEncoder.getPosition() - targetPositionLeft) <= Constants.ClimberConstants.kTolerance) {
					// Yes we are
					atSetPointRight = true;
					System.out.println("setting atSetPint to true");
				}
				
			} else if(Constants.getMode() == Mode.SIM) {

				// We are not at the set point and are in SIM
				pidOutputRight = pidControllerRight.calculate(rightEncoder.getPosition(), targetPositionRight);

				if(pidOutputRight > 12) {
					pidOutputRight = 12;
				} else if(pidOutputRight < -12) {
					pidOutputRight = -12;
				}

				if(pidControllerRight.atSetpoint()) {
					System.out.println("we are at the setpointRight, position: " + rightEncoder.getPosition());
					atSetPointRight = true;
					if(Constants.getMode() == Mode.SIM) {
						rightMotor.setVoltage(0.0);
					} 
				} else {
					// we have not reached the set point yet
					rightMotor.setVoltage(pidOutputRight);
					//System.out.println("adjusting position: " + rightMotor.getEncoder().getPosition());
				}
			}
			
		} else {
			//System.out.println("we are at the setpoint, position: " + rightEncoder.getPosition());

			if(Constants.getMode() == Mode.SIM) {
				rightMotor.setVoltage(0.0);
			}
		}

		// Left

		if(atSetPointLeft == false) {

			if(Constants.getMode() == Mode.REAL) {

				//System.out.println("it is being called, targetPosition: " + targetPosition);
				REVLibError error = leftPIDController.setReference(targetPositionLeft, CANSparkMax.ControlType.kPosition);

				if(error != REVLibError.kOk) {
					System.out.println("ClimberSubsystem::setReference() - Error - could not set the PID controller");
				}

				error = leftPIDController.setReference(targetPositionLeft, CANSparkMax.ControlType.kPosition);

				// Are we close to the target position?
				if(Math.abs(leftEncoder.getPosition() - targetPositionLeft) <= Constants.ClimberConstants.kTolerance) {
					// Yes we are
					atSetPointLeft = true;
					System.out.println("setting atSetPoint to true");
				}

				if(Math.abs(leftEncoder.getPosition() - targetPositionLeft) <= Constants.ClimberConstants.kTolerance) {
					// Yes we are
					atSetPointLeft = true;
					System.out.println("setting atSetPint to true");
				}
				
			} else if(Constants.getMode() == Mode.SIM) {

				// We are not at the set point and are in SIM
				pidOutputLeft = pidControllerLeft.calculate(leftEncoder.getPosition(), targetPositionLeft);

				if(pidOutputLeft > 12) {
					pidOutputLeft = 12;
				} else if(pidOutputLeft < -12) {
					pidOutputLeft = -12;
				}

				if(pidControllerLeft.atSetpoint()) {
					System.out.println("we are at the setpointRight, position: " + leftEncoder.getPosition());
					atSetPointLeft = true;
					if(Constants.getMode() == Mode.SIM) {
						leftMotor.setVoltage(0.0);
					} 
				} else {
					// we have not reached the set point yet
					leftMotor.setVoltage(pidOutputLeft);

					//System.out.println("adjusting position: " + leftMotor.getEncoder().getPosition());
				}
			}
			
		} else {
			//System.out.println("we are at the setpoint, position: " + rightEncoder.getPosition());

			if(Constants.getMode() == Mode.SIM) {
				leftMotor.setVoltage(0.0);
			}
		}

		
	}

	/** resets the zeros of the arms to their current positions */
	/*public void resetZeros() {
		rightEncoder.setPosition(0);
		//leftEncoder.setPosition(0);
	}*/

	/**
	 * @return the Output current
	 */
	public double getLeftMotorOutput() {
		return leftMotor.getOutputCurrent();
	}

	/**
	 * @return the Output current
	 */
	public double getRightMotorOutput() {
		return rightMotor.getOutputCurrent();
	}

	@Override
	public void periodic() {

		//ClimberPosition.setDouble(rightEncoder.getPosition());
		//ClimberTarget.setDouble(targetPosition);

		//Logger.getInstance().recordOutput("Climber/position", rightEncoder.getPosition());
		//Logger.getInstance().recordOutput("Climber/target", targetPosition);

		double leftValue = JoystickUtils.processJoystickInput(operatorController.getLeftY());
		double rightValue = JoystickUtils.processJoystickInput(operatorController.getRightY());

		double maxValue = climberStates.get(kClimberPoses.HIGH)[0];

		if(leftValue > 0.1) {
			// move down
			if(Math.abs( leftMotor.getEncoder().getPosition()) >= 5.0) {
				atSetPointLeft = false;
				targetPositionLeft += leftValue;
			}
		} else if(leftValue < -0.1) {
			// move up
			if(Math.abs( leftMotor.getEncoder().getPosition()) <= maxValue) {
				atSetPointLeft = false;
				this.targetPositionLeft += leftValue;
			}
		}

		//System.out.println("rightValue: " + rightValue);

		if(rightValue > 0.1) {
			// move down
			if(Math.abs( rightMotor.getEncoder().getPosition()) >= 5.0) {
				atSetPointRight = false;
				targetPositionRight += rightValue;
			}
		} else if(rightValue < -0.1) {
			// move up
			if(Math.abs( rightMotor.getEncoder().getPosition()) <= maxValue) {
				atSetPointRight = false;
				targetPositionRight += rightValue;
			}
		}

		Logger.recordOutput("Climber/positionRight", rightEncoder.getPosition());
		Logger.recordOutput("Climber/targetPositionRight", targetPositionRight);

		//entry = climberTable.getEntry("position");

		//double value = entry.getDouble(0.0);

		// Set the value of the shuffleboard
		//climberNTPosition.setDouble(value);
		
		// Set the value of the PID
		setReference();
	}

	public Command SequencedArmPoseCommand(final kClimberPoses state) {
		return runOnce(() -> {
			//setSequencedArmState(/*state*/);
		});
	}

	public Command UnsequencedArmPoseCommand(final kClimberPoses state) {
		return runOnce(() -> {
			//setUnsequencedArmState(/*state*/);
		});
	}

	/*public void setSequencedClimberState(kClimberPoses state) {

		setTargetClimberState(state);

		if (state == kClimberPoses.TUCKED) {
			setReference();
		} else {
			setReference();
		}
	}*/


	public void setTargetClimberState(kClimberPoses state) {

		if(state == kClimberPoses.HIGH && targetClimberState == kClimberPoses.HIGH) {
			targetClimberState = kClimberPoses.TUCKED;
		} else if(state == kClimberPoses.TUCKED && targetClimberState == kClimberPoses.TUCKED) {
			targetClimberState = kClimberPoses.HIGH;
		} else {
			targetClimberState = state;
		}
		
		enableClimber = true;

		targetPositionLeft = climberStates.get(targetClimberState)[0];
		targetPositionRight = climberStates.get(targetClimberState)[0];
		atSetPointRight = false;
		atSetPointLeft = false;

		// get minor speed from map
		// gets the angle values from the hashmap
		//majorArm.setTargetTheta(armStates.get(targetArmState)[0]);
		//setTargetTheta(climberStates.get(targetClimberState)[0]);
		//minorArm.setTargetTheta(armStates.get(targetArmState)[1]);
	}

	/*public void setTargetTheta(double theta) {
		theta = constrain(theta);
		targetTheta = Math.toRadians(theta);
	}*/

	/**
	 * Forces a given angle to be between thw min and max constraints
	 * 
	 * @param theta the angle to be constrained
	 * @return the limited value of theta
	 */
	/*public double constrain(double theta) {

		if (theta >= maxTheta) {
			return maxTheta;

		} else if (theta <= minTheta) {
			return minTheta;
		}

		return theta;
	}*/

	/*public boolean getAtTarget(double deadBand) {

		if (majorArm.getAtTarget(deadBand) && minorArm.getAtTarget(deadBand)) {
			return true;
		}
		return false;
	}*/

	/*public boolean isFullyExtended() {
		return true;
	}

	public boolean isFullyRetracted() {
		return true;
	}*/

	public void increaseTargetPositionRight() {
		targetPositionRight++;
	}

	public void decreaseTargetPositionRight() {
		targetPositionRight--;
	}

	public void increaseTargetPositionLeft() {
		targetPositionLeft++;
	}

	public void decreaseTargetPositionLeft() {
		targetPositionLeft--;
	}

	public double getTargetPositionRight() {
		return this.targetPositionRight;
	}

	public double getTargetPositionLeft() {
		return this.targetPositionLeft;
	}

	public double getPositionRight() {
		//return rightEncoder.getPosition();
		return rightMotor.getEncoder().getPosition();
	}

	public double getPositionLeft() {
		//return leftEncoder.getPosition();
		return leftMotor.getEncoder().getPosition();
	}

	public double getRightOutput() {
		return rightMotor.getAppliedOutput();
	}

	public double getLeftOutput() {
		return leftMotor.getAppliedOutput();
	}

	public double getRightTemp() {
		return (rightMotor.getMotorTemperature() * 9 / 5) + 32;
	}

	public double getLeftTemp() {
		return (leftMotor.getMotorTemperature() * 9 / 5) + 32;
	}

	public double getRightCurrent() {
		return rightMotor.getOutputCurrent();
	}

	public double getLeftCurrent() {
		return leftMotor.getOutputCurrent();
	}

	public void changeLeftTargetPosition(double amountToChange) {
		targetPositionLeft += amountToChange;
	}
	
	public void changeRightTargetPosition(double amountToChange) {
		targetPositionRight += amountToChange;
	}
}
