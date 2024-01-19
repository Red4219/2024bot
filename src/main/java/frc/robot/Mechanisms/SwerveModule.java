// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Mechanisms;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderSimCollection;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Tools.Parts.PIDGains;

public class SwerveModule {
	/** Creates a new SwerveModule. */

	private final CANSparkMax driveMotor;
	private final CANSparkMax turningMotor;

	private final CANCoder absoluteEncoder;
	CANCoderSimCollection simCollection;
	private final RelativeEncoder driveEncoder;

	private final SparkPIDController drivePID;
	private final ProfiledPIDController m_turningPIDController;

	public final double angleZero;

	private final String moduleName;
	private Rotation2d _simulatedAbsoluteEncoderRotation2d = new Rotation2d();

	SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(
			ModuleConstants.ksTurning, ModuleConstants.kvTurning);

	public SwerveModule(
			String moduleName,
			int driveMotorChannel,
			int turningMotorChannel,
			int absoluteEncoderPort,
			double angleZero,
			PIDGains angularPID,
			PIDGains drivePID
			) {

		this.moduleName = moduleName;
		this.angleZero = angleZero;

		// Initialize the motors
		driveMotor = new CANSparkMax(driveMotorChannel, MotorType.kBrushless);

		if(Constants.getMode() == Mode.SIM) {
			REVPhysicsSim.getInstance().addSparkMax(driveMotor, 2.6f, 5676);
		}
		turningMotor = new CANSparkMax(turningMotorChannel, MotorType.kBrushless);

		if(Constants.getMode() == Mode.SIM) {
			REVPhysicsSim.getInstance().addSparkMax(turningMotor, 2.6f, 5676);
		}

		driveMotor.setInverted(true);
		turningMotor.setInverted(false);

		turningMotor.restoreFactoryDefaults();
		driveMotor.restoreFactoryDefaults();

		// Initalize CANcoder
		absoluteEncoder = new CANCoder(absoluteEncoderPort);
		if(Constants.getMode() == Mode.SIM) {
			simCollection = absoluteEncoder.getSimCollection();
		}
		Timer.delay(1);
		absoluteEncoder.configFactoryDefault();
		absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
		absoluteEncoder.configMagnetOffset(-1 * angleZero);
		absoluteEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10, 100);
		absoluteEncoder.clearStickyFaults();

		driveEncoder = driveMotor.getEncoder();
		driveEncoder.setPositionConversionFactor(ModuleConstants.kdriveGearRatioL1 * ModuleConstants.kwheelCircumference); // meters
		driveMotor.getEncoder().setVelocityConversionFactor(
				ModuleConstants.kdriveGearRatioL1
						* ModuleConstants.kwheelCircumference
						* (1d / 60d)); // meters per second

		// Initialize PID's
		this.drivePID = driveMotor.getPIDController();
		this.drivePID.setP(drivePID.kP);
		this.drivePID.setI(drivePID.kI);
		this.drivePID.setD(drivePID.kD);

		m_turningPIDController = new ProfiledPIDController(
			angularPID.kP,
			angularPID.kI,
			angularPID.kD,
			new TrapezoidProfile.Constraints( // radians/s?
					2 * Math.PI * 600, // theoretical is 5676 RPM -> 94*2pi
					2 * Math.PI * 1200));

		this.drivePID.setFF(ModuleConstants.kDriveFeedForward);

		this.drivePID.setFeedbackDevice(driveMotor.getEncoder());

		this.drivePID.setOutputRange(-1, 1);

		// Configure current limits for motors
		driveMotor.setIdleMode(IdleMode.kBrake);
		turningMotor.setIdleMode(IdleMode.kBrake);
		turningMotor.setSmartCurrentLimit(ModuleConstants.kTurnMotorCurrentLimit);
		driveMotor.setSmartCurrentLimit(ModuleConstants.kDriveMotorCurrentLimit);

		m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

		SmartDashboard.putNumber(this.moduleName + " Offset", angleZero);
		SmartDashboard.putString(this.moduleName + " Abs. Status", absoluteEncoder.getLastError().toString());
	}

	// Returns headings of the module
	public double getAbsoluteHeading() {
		return absoluteEncoder.getAbsolutePosition();
	}

	public double getDistanceMeters() {
		return driveEncoder.getPosition();
	}

	// Returns current position of the modules
	public SwerveModulePosition getPosition() {

		if(Constants.getMode() == Mode.SIM){
			double m_distanceMeters = driveEncoder.getPosition();
	
			return new SwerveModulePosition(m_distanceMeters, _simulatedAbsoluteEncoderRotation2d);
		}

		double m_moduleAngleRadians = Math.toRadians(absoluteEncoder.getAbsolutePosition());
		double m_distanceMeters = driveEncoder.getPosition();

		return new SwerveModulePosition(m_distanceMeters, new Rotation2d(m_moduleAngleRadians));
	}

	public void setDesiredState(SwerveModuleState desiredState) {

		double m_moduleAngleRadians = Math.toRadians(absoluteEncoder.getAbsolutePosition());

		if(Constants.getMode() == Mode.SIM) {
			m_moduleAngleRadians = Math.toRadians(desiredState.angle.getDegrees());
			_simulatedAbsoluteEncoderRotation2d = desiredState.angle;
		}

		// Optimize the reference state to avoid spinning further than 90 degrees to
		// desired state
		SwerveModuleState optimizedState = SwerveModuleState.optimize(
				desiredState,
				new Rotation2d(m_moduleAngleRadians));

		final var angularPIDOutput = m_turningPIDController.calculate(m_moduleAngleRadians,
				optimizedState.angle.getRadians());

		final var angularFFOutput = turnFeedForward.calculate(m_turningPIDController.getSetpoint().velocity);

		final var turnOutput = angularPIDOutput + angularFFOutput;

		//if(this.moduleName.equals("RR")) {
			//turningMotor.setVoltage(angularPIDOutput);
			turningMotor.setVoltage(turnOutput);
		//}

		if(Constants.getMode() == Mode.SIM) {
		drivePID.setReference(
			optimizedState.speedMetersPerSecond,
			CANSparkMax.ControlType.kVoltage);
		} else {
			drivePID.setReference(
				optimizedState.speedMetersPerSecond,
				ControlType.kVelocity);
		}

		SmartDashboard.putNumber(this.moduleName + " Optimized Angle", optimizedState.angle.getDegrees());
		SmartDashboard.putNumber(this.moduleName + " PID", angularPIDOutput);
		SmartDashboard.putNumber(this.moduleName + " Turn Output", turnOutput);

		/*Logger.getInstance().recordOutput("Motors/DriveMotorCurrentOutput_" + moduleName, driveMotor.getOutputCurrent());
		Logger.getInstance().recordOutput("Motors/DriveMotorTemp_" + moduleName, driveMotor.getMotorTemperature());
		Logger.getInstance().recordOutput("Motors/TurnMotorCurrentOutput_" + moduleName, turningMotor.getOutputCurrent());
		Logger.getInstance().recordOutput("Motors/TurnMotorTemp_" + moduleName, turningMotor.getMotorTemperature());*/

		Logger.recordOutput("Motors/DriveMotorCurrentOutput_" + moduleName, driveMotor.getOutputCurrent());
		Logger.recordOutput("Motors/DriveMotorTemp_" + moduleName, driveMotor.getMotorTemperature());
		Logger.recordOutput("Motors/TurnMotorCurrentOutput_" + moduleName, turningMotor.getOutputCurrent());
		Logger.recordOutput("Motors/TurnMotorTemp_" + moduleName, turningMotor.getMotorTemperature());
	}

	public void resetEncoders() {
		Timer.delay(.1);
		absoluteEncoder.configFactoryDefault();
		Timer.delay(.1);
		absoluteEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
		Timer.delay(.1);
		absoluteEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Signed_PlusMinus180);
		Timer.delay(.1);
		absoluteEncoder.configMagnetOffset(-1 * angleZero);
		Timer.delay(.1);
		absoluteEncoder.clearStickyFaults();
	}

	public void stopMotors() {
		driveMotor.stopMotor();
		turningMotor.stopMotor();
	}

}
