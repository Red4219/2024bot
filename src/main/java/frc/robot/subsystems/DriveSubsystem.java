// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// DISCLAIMER! THIS DRIVE_SUBSYSTEM HAS MANY BUGS AND SHOULD NOT BE USED AS A REFRANCE!

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Optional;


import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import frc.robot.Tools.GyroIOInputsAutoLogged;


import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Mechanisms.SwerveModule;
import frc.robot.Tools.AutonomousDetail;
//import frc.robot.Tools.GyroIOInputsAutoLogged;
import frc.robot.Tools.GyroIONavX;
import frc.robot.Tools.LimelightHelpers;
import frc.robot.Tools.PhotonVision;
import frc.robot.Tools.PhotonVisionResult;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.Mode;
import frc.robot.Constants.ModuleConstants;

import frc.robot.RobotContainer;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;

public class DriveSubsystem extends SubsystemBase {

	private boolean fieldRelative = true;
	private boolean gyroTurning = false;
	private double targetRotationDegrees;

	private final SwerveModule frontLeft;
	private final SwerveModule frontRight;
	private final SwerveModule rearLeft;
	private final SwerveModule rearRight;

	private SwerveModulePosition[] swervePosition;

	// Initalizing the gyro sensor
	private final AHRS gyro = new AHRS(SPI.Port.kMXP); 
	private final GyroIONavX _gyroIONavX;
	//private final GyroIOInputsAutoLogged _gyroInputs = new GyroIOInputsAutoLogged();
	private AutonomousDetail _autoDetailSelected = null;

	private double xSpeed = 0.0;
	private double ySpeed = 0.0;
	private double rot = 0.0;


	PhotonVisionResult photonVisionResult = null;

	// Odeometry class for tracking robot pose
	SwerveDriveOdometry odometry;

	// test for auto positioning
	HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
		new PIDController(1, 0, 0),
		new PIDController(1, 0, 0),
		new ProfiledPIDController(
			//1,
			1,
			0,
			0,
			new TrapezoidProfile.Constraints(
			//6.28, 
			12.0,
			//3.14
			6.0
			)
		)
	);

	Trajectory trajectory = null;
	Trajectory.State goal = null;

	// PID controller for gyro turning
	private ProfiledPIDController gyroTurnPidController;

	private Field2d field;

	private SwerveDrivePoseEstimator poseEstimator;

	private PhotonVision _photonVision;
	Pose2d photonPose2d;
	boolean _inMotion = false;
	Constants.RobotStatus _robotStatus = Constants.RobotStatus.RobotInit;

	//GenericEntry autoX_Position;
	//GenericEntry autoY_Position;

	double autoX_Position = 0.0;
	double autoY_Position = 0.0;
	boolean autoPositionStatusX = false;
	boolean autoPositionStatusY = false;
	String alliance = "";

	/**
	* Standard deviations of model states. Increase these numbers to trust your model's state estimates less. This
    * matrix is in the form [x, y, theta]ᵀ, with units in meters and radians, then meters.
    */
    private static final Vector<N3> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  
    /**
    * Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
    * less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
    */
    private static final Vector<N3> visionMeasurementStdDevs = VecBuilder.fill(
		Constants.PhotonVisionConstants.visionMeasurementStdDevsX, 
		Constants.PhotonVisionConstants.visionMeasurementStdDevsY, 
		Constants.PhotonVisionConstants.visionMeasurementStdDevsTheta
	);

	private NetworkTableInstance networkTableInstance = NetworkTableInstance.getDefault();

	/** Creates a new DriveSubsystem. */
	public DriveSubsystem() {
		_photonVision = new PhotonVision();
		//gyro.calibrate();
		gyro.reset();
		_gyroIONavX = new GyroIONavX(gyro);

		frontLeft = new SwerveModule(
				"FL",
				ModuleConstants.kFrontLeftDriveMotorPort,
				ModuleConstants.kFrontLeftTurningMotorPort,
				ModuleConstants.kFrontLeftTurningEncoderPort,
				ModuleConstants.kFrontLeftAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains
			);

		frontRight = new SwerveModule(
				"FR",
				ModuleConstants.kFrontRightDriveMotorPort,
				ModuleConstants.kFrontRightTurningMotorPort,
				ModuleConstants.kFrontRightTurningEncoderPort,
				ModuleConstants.kFrontRightAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains
			);

		rearLeft = new SwerveModule(
				"RL",
				ModuleConstants.kRearLeftDriveMotorPort,
				ModuleConstants.kRearLeftTurningMotorPort,
				ModuleConstants.kRearLeftTurningEncoderPort,
				ModuleConstants.kRearLeftAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains
			);

		rearRight = new SwerveModule(
				"RR",
				ModuleConstants.kRearRightDriveMotorPort,
				ModuleConstants.kRearRightTurningMotorPort,
				ModuleConstants.kRearRightTurningEncoderPort,
				ModuleConstants.kRearRightAngleZero,
				ModuleConstants.kModuleTurningGains,
				ModuleConstants.kModuleDriveGains
			);

		swervePosition = new SwerveModulePosition[] {
				frontLeft.getPosition(),
				frontRight.getPosition(),
				rearLeft.getPosition(),
				rearRight.getPosition()
		};

		//gyro = new WPI_Pigeon2(DriveConstants.kPigeonPort, Constants.kCanivoreCANBusName);
		//gyro.setYaw(0);

		odometry = new SwerveDriveOdometry(
				DriveConstants.kDriveKinematics,
				gyro.getRotation2d(),
				swervePosition);

		field = new Field2d();

		gyroTurnPidController = new ProfiledPIDController(
				DriveConstants.kGyroTurningGains.kP,
				DriveConstants.kGyroTurningGains.kI,
				DriveConstants.kGyroTurningGains.kD,
				new TrapezoidProfile.Constraints(
						DriveConstants.kMaxTurningVelocityDegrees,
						DriveConstants.kMaxTurningAcceleratonDegrees));

		gyroTurnPidController.enableContinuousInput(-180, 180);
		gyroTurnPidController.setTolerance(DriveConstants.kGyroTurnTolerance);

		/*poseEstimator = new SwerveDrivePoseEstimator(
				DriveConstants.kDriveKinematics,
				gyro.getRotation2d(),
				swervePosition,
				new Pose2d());*/

		poseEstimator = new SwerveDrivePoseEstimator(
				DriveConstants.kDriveKinematics,
				gyro.getRotation2d(),
				swervePosition,
				new Pose2d(),
				stateStdDevs,
				visionMeasurementStdDevs
			);
		

		targetRotationDegrees = 0;

		ShuffleboardTab autoTab = Shuffleboard.getTab("Autonomous");

		autoTab.addDouble("AutoX Position", this::getAutoX_Position);
		autoTab.addDouble("AutoY Position", this::getAutoY_Position);

		autoTab.addBoolean("AutoX Status", this::getAutoPositionStatusX);
		autoTab.addBoolean("AutoY Status", this::getAutoPositionStatusY);

		if(DriverStation.getAlliance().isPresent()) {
			if (DriverStation.getAlliance().get() == Alliance.Blue) {
				alliance = "Blue";
			} else {
				alliance = "Red";
			}
		} else {
			alliance = "Blue";
		}

		autoTab.addString("Alliance", this::getAlliance);
	}

	public PhotonVision getPhotonVision() {
		return _photonVision;
	}

	@Override
	public void periodic() {
		// This method will be called once per scheduler run
		
		updateOdometry();

		if (DriverStation.isDisabled()) {
			// frontLeft.resetEncoders();
			// frontRight.resetEncoders();
			// rearLeft.resetEncoders();
			// rearRight.resetEncoders();
		}

		SmartDashboard.putNumber("FL Absolute", frontLeft.getAbsoluteHeading());
		SmartDashboard.putNumber("FR Absolute", frontRight.getAbsoluteHeading());
		SmartDashboard.putNumber("RL Absolute", rearLeft.getAbsoluteHeading());
		SmartDashboard.putNumber("RR Absolute", rearRight.getAbsoluteHeading());

		//System.out.println(rearLeft.getAbsoluteHeading());
		//System.out.println(frontRight.getAbsoluteHeading());

		SmartDashboard.putNumber("FL Offset Check", frontLeft.getAbsoluteHeading() + frontLeft.angleZero);
		SmartDashboard.putNumber("FR Offset Check", frontRight.getAbsoluteHeading() + frontRight.angleZero);
		SmartDashboard.putNumber("RL Offset Check", rearLeft.getAbsoluteHeading() + rearLeft.angleZero);
		SmartDashboard.putNumber("RR Offset Check", rearRight.getAbsoluteHeading() + rearRight.angleZero);

		SmartDashboard.putNumber("Gyro yaw", gyro.getYaw());
		SmartDashboard.putNumber("Gyro pitch", gyro.getPitch());
		SmartDashboard.putNumber("Gyro roll", gyro.getRoll());

		SmartDashboard.putNumber("FL Meters", frontLeft.getDistanceMeters());
		SmartDashboard.putNumber("FR Meters", frontRight.getDistanceMeters());
		SmartDashboard.putNumber("RL Meters", rearLeft.getDistanceMeters());
		SmartDashboard.putNumber("RR Meters", rearRight.getDistanceMeters());

		SmartDashboard.putData("field", field);
		SmartDashboard.putNumber("2D Gyro", odometry.getPoseMeters().getRotation().getDegrees());
		SmartDashboard.putNumber("2D X", getPose().getX());
		SmartDashboard.putNumber("2D Y", getPose().getY());
		SmartDashboard.putBoolean("PV Status", _photonVision.isConnected());

		if (_robotStatus == Constants.RobotStatus.DisabledPeriodic && _autoDetailSelected != null) {
			if (DriverStation.getAlliance().get() == Alliance.Blue) {
				// if(poseEstimator.getEstimatedPosition().getX() < )
				alliance = "Blue";
				if (photonVisionResult != null) {
					Pose2d pose = photonVisionResult.pose3d().toPose2d();

					if (pose.getX() >= _autoDetailSelected.startXblue + _autoDetailSelected.startXTolerance || pose.getX() <= _autoDetailSelected.startXblue - _autoDetailSelected.startXTolerance) {
						autoX_Position = pose.getX() - _autoDetailSelected.startXblue;
						autoPositionStatusX = false;
					} else {
						autoPositionStatusX = true;
					}

					if (pose.getY() >= _autoDetailSelected.startYblue + _autoDetailSelected.startYTolerance || pose.getY() <= _autoDetailSelected.startYblue - _autoDetailSelected.startYTolerance) {
						autoY_Position = pose.getY() - _autoDetailSelected.startYblue;
						autoPositionStatusY = false;
					} else {
						autoPositionStatusY = true;
					}
				}
			} else {
				alliance = "Red";
				if (photonVisionResult != null) {
					Pose2d pose = photonVisionResult.pose3d().toPose2d();

					if (pose.getX() >= _autoDetailSelected.startXred + _autoDetailSelected.startXTolerance || pose.getX() <= _autoDetailSelected.startXred - _autoDetailSelected.startXTolerance) {
						autoX_Position = pose.getX() - _autoDetailSelected.startXred;
						autoPositionStatusX = false;
					} else {
						autoPositionStatusX = true;
					}

					if (pose.getY() >= _autoDetailSelected.startYred + _autoDetailSelected.startYTolerance || pose.getY() <= _autoDetailSelected.startYred - _autoDetailSelected.startYTolerance) {
						autoY_Position = pose.getY() - _autoDetailSelected.startYred;
						autoPositionStatusY = false;
					} else {
						autoPositionStatusY = true;
					}
				}
			}
		}
	}

	public void setAutoCommandSelected(Command autoCommand) {
		if(autoCommand != null) {
			_autoDetailSelected = Constants.AutonomousRoutines.get(autoCommand.getName());
			System.out.println("auto selected: " + autoCommand.getName());
		}
	}

	// region getters
	public double getHeading() {
		if(Constants.getMode() == Mode.SIM) {
			return gyro.getRotation2d().getDegrees();
		}

		return gyro.getRotation2d().getDegrees();
	}

	public double getHeading360() {
		if(Constants.getMode() == Mode.SIM) {
			return (gyro.getRotation2d().getDegrees() % 360);
		}
		
		return (gyro.getRotation2d().getDegrees() % 360);
	}

	public double getRoll() {
		return gyro.getRoll();
	}

	public double getPitch() {
		return gyro.getPitch();
	}

	/*public double getTurnRate() {
		return gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
	}*/

	public Pose2d getPose() {
		return odometry.getPoseMeters();
	}

	public Pose2d getPoseEstimatorPose2d() {
		//System.out.println("getPoseEstimatorPose2d called");
		return poseEstimator.getEstimatedPosition();
	}

	public void resetOdometry(Pose2d pose) {

		odometry.resetPosition(
			(Constants.getMode() == Mode.SIM) ? pose.getRotation() : gyro.getRotation2d(),
			swervePosition,
			pose
		);

		poseEstimator.resetPosition(
			(Constants.getMode() == Mode.SIM) ? pose.getRotation() : gyro.getRotation2d(),
			swervePosition,
			pose
		);

		_photonVision.setReferencePose(pose);

		Logger.recordOutput("Odometry/Robot", odometry.getPoseMeters());
		Logger.recordOutput("Estimator/Robot", poseEstimator.getEstimatedPosition());
	}
	// endregion

	/*public void resetVisionOdometry(Pose2d pose) {
		_photonVision.setReferencePose(pose);
	}*/

	// region setter

	public void lockWheels() {
		double rot = DriveConstants.kMaxRPM;

		SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
				new ChassisSpeeds(0, 0, rot));

		SwerveDriveKinematics.desaturateWheelSpeeds(
				swerveModuleStates, 0);

		setModuleStates(swerveModuleStates);
	}

	public void robotCentricDrive(double xSpeed, double ySpeed, double rot) {
		setFieldCentric(false);
		drive(xSpeed, ySpeed, rot);
		setFieldCentric(true);
	}

	public void drive(double xSpeed, double ySpeed, double rot) {
		drive(xSpeed, ySpeed, rot, false, false);
	}

	public void drive(double xSpeed, double ySpeed, double rot, boolean isTurbo, boolean isSneak) {

		double maxSpeed;

		if (isSneak) {
			maxSpeed = DriveConstants.kMaxSneakMetersPerSecond;
		} else if (isTurbo) {
			maxSpeed = DriveConstants.kMaxTurboMetersPerSecond;
		} else {
			maxSpeed = DriveConstants.kMaxSpeedMetersPerSecond;
		}

		// Apply deadbands to inputs
		xSpeed *= maxSpeed;
		ySpeed *= maxSpeed;

		if (gyroTurning) {
			targetRotationDegrees += rot;
			rot = gyroTurnPidController.calculate(getHeading360(), targetRotationDegrees);
		} else {
			rot *= DriveConstants.kMaxRPM;
		}

		this.xSpeed = xSpeed;
		this.ySpeed = ySpeed;
		this.rot = rot;

		/*SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
				fieldRelative
						? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d())
						: new ChassisSpeeds(xSpeed, ySpeed, rot));*/

		SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
				ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d()));
		
		setModuleStates(swerveModuleStates);
	}

	public ChassisSpeeds getChassisSpeedsRobotRelative() {
		return ChassisSpeeds.fromRobotRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getRotation2d());
	}

	public void setChassisSpeedsRobotRelative(ChassisSpeeds chassisSpeeds ){
		SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
		//setModuleStates(swerveModuleStates);

		if(Constants.getMode() == Mode.SIM) {
			int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
			SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
			angle.set(angle.get() + -chassisSpeeds.omegaRadiansPerSecond);
		}

		frontLeft.setDesiredState(swerveModuleStates[0]);
		frontRight.setDesiredState(swerveModuleStates[1]);
		rearLeft.setDesiredState(swerveModuleStates[2]);
		rearRight.setDesiredState(swerveModuleStates[3]);

		Logger.recordOutput("SwerveModuleStates/Setpoints", swerveModuleStates);
	}

	public void setModuleStates(SwerveModuleState[] desiredStates) {
		SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, ModuleConstants.kMaxModuleSpeedMetersPerSecond);

		if(desiredStates[0].speedMetersPerSecond == 0.0
			&& desiredStates[1].speedMetersPerSecond == 0.0
			&& desiredStates[2].speedMetersPerSecond == 0.0
			&& desiredStates[3].speedMetersPerSecond == 0.0
		) {
			_inMotion = false;
		} else {
			_inMotion = true;
		}		
		
		if(Constants.getMode() == Mode.SIM) {

			ChassisSpeeds chassisSpeeds = DriveConstants.kDriveKinematics.toChassisSpeeds(
				desiredStates[0], desiredStates[1], desiredStates[2], desiredStates[3]
			);

			int dev = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
			SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(dev, "Yaw"));
			angle.set(angle.get() + -chassisSpeeds.omegaRadiansPerSecond);
		}

		frontLeft.setDesiredState(desiredStates[0]);
		frontRight.setDesiredState(desiredStates[1]);
		rearLeft.setDesiredState(desiredStates[2]);
		rearRight.setDesiredState(desiredStates[3]);

		Logger.recordOutput("SwerveModuleStates/Setpoints", desiredStates);
	}

	public void updateOdometry() {
		swervePosition = new SwerveModulePosition[] {
				frontLeft.getPosition(),
				frontRight.getPosition(),
				rearLeft.getPosition(),
				rearRight.getPosition()
		};

		// For some reason, the code below is preventing rotation in SIM
		if(Constants.getMode() == Mode.SIM) {

			odometry.update(
				Rotation2d.fromDegrees(getHeading()),
				swervePosition);

			poseEstimator.update(
				Rotation2d.fromDegrees(getHeading()),
				swervePosition
			);

		} else {
			odometry.update(
				gyro.getRotation2d(), 
				swervePosition
			);

			poseEstimator.update(
				gyro.getRotation2d(), 
				swervePosition
			);
		}

		//_photonVision.setReferencePose(poseEstimator.getEstimatedPosition());

		photonVisionResult = _photonVision.getPose(poseEstimator.getEstimatedPosition());
		
		if(photonVisionResult.targetFound()) {
			photonPose2d = photonVisionResult.pose3d().toPose2d();

			poseEstimator.addVisionMeasurement(
				photonPose2d,
				Timer.getFPGATimestamp() - photonVisionResult.imageCaptureTime(),
				visionMeasurementStdDevs
			);
			//System.out.println("adding Vision Measurement");
			Logger.recordOutput("PhotonVisionEstimator/Robot", photonPose2d);
		}

		/*if(_photonVision != null) {

			// Check if we are not moving, update the odometry with the estimated field position
			if(_inMotion == false) {
				//System.out.println("we are not in motion");

				if(photonVisionResult.targetFound()) {
					//System.out.println("updating the pose from photonvision");
					poseEstimator.resetPosition(
						(Constants.getMode() == Mode.SIM) ? photonPose2d.getRotation() : gyro.getRotation2d(),
						swervePosition,
						photonPose2d
					);
				}
			} 
		}*/

		field.setRobotPose(odometry.getPoseMeters());

		Logger.recordOutput("Odometry/Robot", odometry.getPoseMeters());
		Logger.recordOutput("Estimator/Robot", poseEstimator.getEstimatedPosition());
		

		//_gyroIONavX.updateInputs(_gyroInputs);
        //Logger.getInstance().processInputs("Drive/Gyro", _gyroInputs);
		//Logger.processInputs("Drive/Gyro", _gyroInputs);
	}

	public void resetEncoders() {
		frontLeft.resetEncoders();
		rearLeft.resetEncoders();
		frontRight.resetEncoders();
		rearRight.resetEncoders();
	}

	public void zeroHeading() {
		gyro.reset();
	}

	public void setHeading(double heading) {
		System.out.println("setHeading called");
		//gyro.setYaw(heading);
	}

	public Command toggleFieldCentric() {
		return runOnce(() -> {
			fieldRelative = !fieldRelative;
		});
	}

	public void setFieldCentric(boolean fieldCentric) {
		fieldRelative = fieldCentric;
	}

	public void stopMotors() {
		frontLeft.stopMotors();
		frontRight.stopMotors();
		rearLeft.stopMotors();
		rearRight.stopMotors();
	}

	public void setRobotStatus(Constants.RobotStatus status) {
		_robotStatus = status;
	}

	// endregion

	public void generateTrajectory() {


		//NetworkTable table = networkTableInstance.getTable("/photonvision");
		//boolean asdf = table.containsSubTable("cam1");

		NetworkTable camtable = networkTableInstance.getTable("/photonvision/cam1");
		//boolean asdff = camtable.containsKey("targetYaw");
		//DoubleTopic topic = camtable.getDoubleTopic("targetYaw");
		NetworkTableEntry entry = camtable.getEntry("targetYaw");

		double value = entry.getDouble(0.0);

		//DoubleEntry entry = topic
		//DoubleTopic dblTopic = table.getDoubleTopic("X");
		//DoubleTopic dblTopic = networkTableInstance.getDoubleTopic("/photonvision");




		// 2018 cross scale auto waypoints.
		var sideStart = new Pose2d(
			odometry.getPoseMeters().getX(), 
			odometry.getPoseMeters().getY(),
			odometry.getPoseMeters().getRotation()
		);

		if(DriverStation.getAlliance().isPresent()) {

			if (DriverStation.getAlliance().get() == Alliance.Blue) {
				alliance = "Blue";
			} else {
				alliance = "Red";
			}
		}	 
		
		var crossScale = new Pose2d(
			//Units.feetToMeters(9.0),
			//2.0,
			(DriverStation.getAlliance().get() == Alliance.Blue) ? 2.0 : 13.0,
			//Units.feetToMeters(23.0),
			//(DriverStation.getAlliance().get() == Alliance.Blue) ? 1.09 : 8.1026 - 1.09,
			1.09,
			Rotation2d.fromDegrees(0.0)
		);
	
		var interiorWaypoints = new ArrayList<Translation2d>();
		/*interiorWaypoints.add(
			new Translation2d(
				Units.feetToMeters(20.00), 
				Units.feetToMeters(20.00)
			)
		);*/
		/*interiorWaypoints.add(
			new Translation2d(
				Units.feetToMeters(21.04),
				Units.feetToMeters(18.23)
			)
		);*/
	
		TrajectoryConfig config = new TrajectoryConfig(
			Units.feetToMeters(12),
			Units.feetToMeters(12)
		);

		//config.setReversed(true);
	
		trajectory = TrajectoryGenerator.generateTrajectory(
			sideStart,
			interiorWaypoints,
			crossScale,
			config
		);

		goal = trajectory.sample(trajectory.getTotalTimeSeconds());
	}

	public void goToPose() {

		if(trajectory == null) {
			generateTrajectory();
		}

		System.out.println("go to pose called, time is: " + trajectory.getTotalTimeSeconds());

		// Get the adjusted speeds. Here, we want the robot to be facing
		// 180 degrees (in the field-relative coordinate system).
		ChassisSpeeds adjustedSpeeds = holonomicDriveController.calculate(
  			//getPose(),
			getPoseEstimatorPose2d(),
			goal,
			Rotation2d.fromDegrees(0.0)
		);

		SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(adjustedSpeeds);

		setModuleStates(moduleStates);
	}

	public boolean getAutoPositionStatusX() {
		return autoPositionStatusX;
	}

	public boolean getAutoPositionStatusY() {
		return autoPositionStatusY;
	}

	public String getAlliance() {
		return alliance;
	}

	public double getAutoX_Position() {
		return autoX_Position;
	}

	public double getAutoY_Position() {
		return autoY_Position;
	}
}
