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
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ClimberConstants.kClimberPoses;

public class ClimberSubsystem extends SubsystemBase {

	/** used to track the state of the climber */
	private kClimberPoses targetClimberState;
	private boolean enableClimber;

	private CANSparkMax rightMotor, leftMotor;
	public SparkPIDController rightPIDController;
	private RelativeEncoder rightEncoder, leftEncoder;

	private ProfiledPIDController profiledPIDController;
	private PIDController pidController;

	private boolean invertLeader = true;

	private boolean atSetPoint = true;
	private double targetPosition = 0.0;
	double pidOutput = 0.0;

	HashMap<kClimberPoses, double[]> climberStates = Constants.ClimberConstants.kClimberStatesMap;

	private static ShuffleboardTab ClimberTab = Shuffleboard.getTab("Climber");
	private static GenericEntry ClimberPosition = ClimberTab.addPersistent("Climber Position", 0).getEntry();
	private static GenericEntry ClimberTarget = ClimberTab.addPersistent("Climber Target", 0).getEntry();

	private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();
	private static GenericEntry climberNTPosition = ClimberTab.addPersistent("Climber NT Value", 0).getEntry();

	NetworkTable climberTable = networkTableInstance.getTable("/climber");
	NetworkTableEntry entry;

	public ClimberSubsystem() {
		rightMotor = new CANSparkMax(frc.robot.Constants.ClimberConstants.kRightPort, MotorType.kBrushless);
		leftMotor = new CANSparkMax(frc.robot.Constants.ClimberConstants.kLeftPort, MotorType.kBrushless);

		if(Constants.getMode() == Mode.SIM) {
			REVPhysicsSim.getInstance().addSparkMax(rightMotor, 2.6f, 5676);
			REVPhysicsSim.getInstance().addSparkMax(leftMotor, 2.6f, 5676);
		}

		rightMotor.restoreFactoryDefaults();
		leftMotor.restoreFactoryDefaults();

		rightMotor.setIdleMode(IdleMode.kBrake);
		leftMotor.setIdleMode(IdleMode.kBrake);

		rightEncoder = rightMotor.getEncoder();
		leftEncoder = leftMotor.getEncoder();

		resetZeros();

		// Tells the motors to automatically convert degrees to rotations
		if(Constants.getMode() == Mode.REAL) {
			rightEncoder.setPositionConversionFactor((2 * Math.PI) / frc.robot.Constants.ClimberConstants.kClimberTicks);
			leftEncoder.setPositionConversionFactor((2 * Math.PI) / frc.robot.Constants.ClimberConstants.kClimberTicks);
			rightEncoder.setVelocityConversionFactor((2 * Math.PI) / frc.robot.Constants.ClimberConstants.kClimberTicks);
			leftEncoder.setVelocityConversionFactor((2 * Math.PI) / frc.robot.Constants.ClimberConstants.kClimberTicks);
		} else {
			rightEncoder.setPositionConversionFactor((2 * Math.PI) / frc.robot.Constants.ArmConstants.kMajorArmTicks);
			leftEncoder.setPositionConversionFactor((2 * Math.PI) / frc.robot.Constants.ArmConstants.kMajorArmTicks);
			rightEncoder.setVelocityConversionFactor((2 * Math.PI) / frc.robot.Constants.ArmConstants.kMajorArmTicks);
			leftEncoder.setVelocityConversionFactor((2 * Math.PI) / frc.robot.Constants.ArmConstants.kMajorArmTicks);
		}

		
		
		//kMajorArmTicks

		/**
		 * In order to use PID functionality for a controller, a SparkMaxPIDController
		 * object is constructed by calling the getPIDController() method on an existing
		 * CANSparkMax object
		 */
		rightPIDController = rightMotor.getPIDController();
		rightPIDController.setFeedbackDevice(rightMotor.getEncoder());

		// tells the pid controller on the arms to use trapezoidal constraints 
		rightPIDController.setSmartMotionAccelStrategy(AccelStrategy.kTrapezoidal, 0);

		rightPIDController.setSmartMotionAllowedClosedLoopError(0, 0);

		//rightMotor.setInverted(invertLeader);
		//leftMotor.follow(rightMotor, true);

		setMaxOutput(frc.robot.Constants.ClimberConstants.kClimberCurrentLimit);

		rightPIDController.setP(Constants.ClimberConstants.kClimberGains.kP);
		rightPIDController.setI(Constants.ClimberConstants.kClimberGains.kI);
		rightPIDController.setD(Constants.ClimberConstants.kClimberGains.kD);

		rightPIDController.setIZone(0);
		rightPIDController.setFF(0);

		//rightMotor.setSmartCurrentLimit(frc.robot.Constants.ClimberConstants.kClimberCurrentLimit);
		rightMotor.setSecondaryCurrentLimit(Constants.ArmConstants.kArmCurrentLimit + 3);

		rightPIDController.setSmartMotionMaxVelocity(Constants.ArmConstants.kMaxMajorVelRadiansPerSec, 0);
		rightPIDController.setSmartMotionMinOutputVelocity(0, 0);
		//rightPIDController.setSmartMotionMaxAccel(Constants.ArmConstants. maxAccel, 0);

		rightPIDController.setOutputRange(-Constants.ArmConstants.kMajorPIDOutputLimit, Constants.ArmConstants.kMajorPIDOutputLimit);

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

			pidController = new PIDController(1,.5,.5);
			// Sets the error tolerance to 5, and the error derivative tolerance to 10 per second
			pidController.setTolerance(Constants.ClimberConstants.kTolerance, 10);
		//}

		
		// Listen for changes
		DoubleSubscriber positionSubscriber = climberTable.getDoubleTopic("position").subscribe(0.0);

		NetworkTableInstance.getDefault().addListener(
			positionSubscriber,
			EnumSet.of(NetworkTableEvent.Kind.kValueAll), 
			event -> {
				System.out.println("the value changed " + event.valueData.value.getDouble());

				targetPosition = event.valueData.value.getDouble();

				atSetPoint = false;
			}
		);
	}

	public boolean isAtSetPoint() {
		return atSetPoint;
	}

	/**
	 * sets the maximum output of the pid controller
	 * 
	 * @param maxOutput the max value that can be sent to the motor (0 - 1)
	 */
	public void setMaxOutput(double maxOutput) {
		rightPIDController.setOutputRange(-maxOutput, maxOutput);
	}

	public void setSmartCurrentLimit(int limit) {
		rightMotor.setSmartCurrentLimit(limit);
		leftMotor.setSmartCurrentLimit(limit);
		rightMotor.setSecondaryCurrentLimit(limit + 3);
		leftMotor.setSecondaryCurrentLimit(limit + 3);
	}

	/** used to disable the motors for rezeroing */
	public void toggleMotors() {
		/*isRunning = !isRunning;

		if (isRunning) {
			rightMotor.stopMotor();
			leftMotor.stopMotor();
		} else {
			setReference();
		}*/
	}

	/** Sets the pid referance point to the target theta of the segment */
	public void setReference() {
	
		if(atSetPoint == false) {

			if(Constants.getMode() == Mode.REAL) {

				//System.out.println("it is being called, targetPosition: " + targetPosition);
				REVLibError error = rightPIDController.setReference(targetPosition, CANSparkMax.ControlType.kPosition);

				if(error != REVLibError.kOk) {
					System.out.println("ClimberSubsystem::setReference() - Error - could not set the PID controller");
				}

				// Are we close to the target position?
				if(Math.abs(rightEncoder.getPosition() - targetPosition) <= Constants.ClimberConstants.kTolerance) {
					// Yes we are
					atSetPoint = true;
					System.out.println("setting atSetPint to true");
				}
				
			} else if(Constants.getMode() == Mode.SIM) {

				// We are not at the set point and are in SIM

				pidOutput = pidController.calculate(rightEncoder.getPosition(), targetPosition);

				if(pidOutput > 12) {
					pidOutput = 12;
				} else if(pidOutput < -12) {
					pidOutput = -12;
				}

				if(pidController.atSetpoint()) {
					System.out.println("we are at the setpoint, position: " + rightEncoder.getPosition());
					atSetPoint = true;
					if(Constants.getMode() == Mode.SIM) {
						rightMotor.setVoltage(0.0);
					} 
				} else {
					// we have not reached the set point yet
					rightMotor.setVoltage(pidOutput);
					System.out.println("adjusting position: " + rightEncoder.getPosition());
				}
			}
			
		} else {
			//System.out.println("we are at the setpoint, position: " + rightEncoder.getPosition());

			if(Constants.getMode() == Mode.SIM) {
				rightMotor.setVoltage(0.0);
			}
		}

		
	}

	/** resets the zeros of the arms to their current positions */
	public void resetZeros() {
		rightEncoder.setPosition(0);
		leftEncoder.setPosition(0);
	}

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

		ClimberPosition.setDouble(rightEncoder.getPosition());
		ClimberTarget.setDouble(targetPosition);

		//Logger.getInstance().recordOutput("Climber/position", rightEncoder.getPosition());
		//Logger.getInstance().recordOutput("Climber/target", targetPosition);

		Logger.recordOutput("Climber/position", rightEncoder.getPosition());
		Logger.recordOutput("Climber/target", targetPosition);

		entry = climberTable.getEntry("position");

		double value = entry.getDouble(0.0);

		// Set the value of the shuffleboard
		climberNTPosition.setDouble(value);
		
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

	public void setSequencedClimberState(kClimberPoses state) {

		setTargetClimberState(state);

		if (state == kClimberPoses.TUCKED) {
			//minorArm.setReference();
			setReference();
		} else {
			//majorArm.setReference();
			setReference();
		}
	}


	public void setTargetClimberState(kClimberPoses state) {
		targetClimberState = state;
		enableClimber = true;

		targetPosition = climberStates.get(targetClimberState)[0];
		atSetPoint = false;

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

	public boolean isFullyExtended() {
		return true;
	}

	public boolean isFullyRetracted() {
		return true;
	}
	
}
