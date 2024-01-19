// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.Map;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Constants.ArmConstants.kArmPoses;
import frc.robot.Tools.AutonomousDetail;
import frc.robot.Tools.Parts.PIDGains;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {

	public static class ModuleConstants {

		// Current limits for the wheels
		public static final int kTurnMotorCurrentLimit = 25;
		public static final int kDriveMotorCurrentLimit = 35;

		// Constants set for the _SDS MK4i_ and MK4
		public static final double kdriveGearRatioL1 = 1d / 8.14;
		public static final double kdriveGearRatioL2 = 1d / 6.75;
		public static final double kdriveGearRatioL3 = 1d / 6.12;
		public static final double kdriveGearRatioL4 = 1d / 5.14;
		public static final double kturnGearRatio = 1d / (150d / 7d);

		public static final double kwheelCircumference = Units.inchesToMeters(4) * Math.PI;

		// The max speed the modules are capable of
		public static final double kMaxModuleSpeedMetersPerSecond = Units.feetToMeters(14.5);

		public static final double ksVolts = .1;
		public static final double kDriveFeedForward = .2;

		// TODO: Retune feedforward values for turning
		public static final double kvTurning = .43205;
		public static final double ksTurning = .17161; // Tuned February 2, 2023

		// NEO drive motor CAN ID's
		public static final int kFrontLeftDriveMotorPort = 1;
		public static final int kFrontRightDriveMotorPort = 5;
		public static final int kRearLeftDriveMotorPort = 4;
		public static final int kRearRightDriveMotorPort = 8;

		// NEO turning motor CAN ID's
		public static final int kFrontLeftTurningMotorPort = 2;
		public static final int kFrontRightTurningMotorPort = 6;
		public static final int kRearLeftTurningMotorPort = 3;
		public static final int kRearRightTurningMotorPort = 7;

		// CANcoder CAN ID's
		public static final int kFrontLeftTurningEncoderPort = 9;
		public static final int kFrontRightTurningEncoderPort = 10;
		public static final int kRearLeftTurningEncoderPort = 12;
		public static final int kRearRightTurningEncoderPort = 11;

		// Offset angle for absolute encoders (find this using CTRE client)
		public static final double kFrontLeftAngleZero = 90;
		public static final double kFrontRightAngleZero = 163.5;
		public static final double kRearLeftAngleZero = -78.3;
		public static final double kRearRightAngleZero = -165.05;

		public static final PIDGains kModuleDriveGains = new PIDGains(.1, 0, 0);

		//public static final PIDGains kModuleTurningGains = new PIDGains(6.5, .25, .15);
		//public static final PIDGains kModuleTurningGains = new PIDGains(1.0, 0.0, 0.1);
		public static final PIDGains kModuleTurningGains = new PIDGains(6.5, 0.25, 0.15);
	}

	public static class DriveConstants {

		public static final double kMaxSneakMetersPerSecond = 1.0;
		public static final double kMaxSpeedMetersPerSecond = 4.5;
		public static final double kMaxTurboMetersPerSecond = 8.0;

		// this sets turning speed (keep this low)
		public static final double kMaxRPM = 10;

		//public static final int kPigeonPort = 20;

		public static final double kBumperToBumperWidth = Units.inchesToMeters(31);

		public static final double kTrackWidth = Units.inchesToMeters(24); // in meters!
		public static final double kWheelBase = Units.inchesToMeters(24); // in meters!

		public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
				new Translation2d(kWheelBase / 2, kTrackWidth / 2), // FL
				new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // FR
				new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // RL
				new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // RR

		public static final boolean kGyroReversed = true;
		//public static final boolean kFieldCentric = true;
		//public static final boolean kGyroTuring = false;

		public static final PIDGains kGyroTurningGains = new PIDGains(.025, 0, 0);
		public static final double kMaxTurningVelocityDegrees = 20;
		public static final double kMaxTurningAcceleratonDegrees = 10;
		public static final double kGyroTurnTolerance = 2;

	}

	/**
	 * The constants pertaining to Autonoumus
	 */
	public static class AutoConstants {

		public static class PathPLannerConstants {

			// PID constants for path planner (these control drive direction not reaching
			// target wheel speeds)
			//public static final PIDGains kPPDriveGains = new PIDGains(8.5, 0, 0);
			//public static final PIDGains kPPTurnGains = new PIDGains(3.5, 0, 0);

			public static final PIDConstants kPPDriveConstants = new PIDConstants(8.5, 0, 0);
			public static final PIDConstants kPPTurnConstants = new PIDConstants(3.5, 0, 0);

			public static final double kPPMaxVelocity = 4.00;
			public static final double kPPMaxAcceleration = 2.50;

			public static final double kMaxModuleSpeed = 4.5; // Max module speed, in m/s
			public static final double kDriveBaseRadius = 0.4; // Drive base radius in meters. Distance from robot center to furthest module.
		}

		//public static final double kScoreSequenceDropTime = 3; // in seconds

		public static final PIDGains kTurnCommandGains = new PIDGains(.004, 0.0003, 0);
		//public static final double kTurnCommandMaxVelocity = 1;
		//public static final double kTurnCommandMaxAcceleration = 1;
		public static final double kTurnCommandToleranceDeg = 0.5;
		public static final double kTurnCommandRateToleranceDegPerS = 0;

		public static final double kBalnaceCommandDeadbandDeg = 2;
		public static final PIDGains kBalanceCommandGains = new PIDGains(.006, 0, 0);
		public static final double kMaxBalancingVelocity = 1000;
		public static final double kMaxBalancingAcceleration = 5000;

		public static final double kAimTargetTolerance = 0.5;

	}

	/**
	 * The constants pertaining to the drive station
	 */
	public static class OperatorConstants {
		public static final int kDriveJoystickPort = 0;
		public static final int kTurnJoystickPort = 1;
		public static final int kOperatorControllerPort = 2;
		public static final int kProgrammerControllerPort = 3;

		public static final double KDeadBand = .125;
		// this is the number that the joystick input will be raised to
		public static final double kJoystickPow = 2.5;
	}

	/**
	 * The constants pertaining to Arm (and sub arms)
	 */
	public static class ArmConstants {

		// NEO turning motor CAN ID's
		public static final int kRightMajorArmPort = 13;
		public static final int kLeftMajorArmPort = 14;
		public static final int kRightMinorArmPort = 15;
		public static final int kLeftMinorArmPort = 16;

		public static final int kMajorArmGearBoxRatio = 100;
		public static final int kMinorArmGearBoxRatio = 100;

		public static final int kMajorArmBeltRatio = 2 / 1;
		public static final int kMinorArmBeltRatio = 1;

		/**
		 * the total number of motor rotations for one 360 degree rotation of the arm
		 */
		public static final int kMajorArmTicks = kMajorArmGearBoxRatio * kMajorArmBeltRatio;
		public static final int kMinorArmTicks = kMinorArmGearBoxRatio * kMinorArmBeltRatio;

		/**
		 * The radius of each arms rotation in inches (from center of rotation to next
		 * arms center of rotation)
		 */
		//public static final int kMajorArmLength = 38;
		//public static final int kMinorArmLength = 23;

		// current limits of the arms
		public static final int kMajorArmCurrentLimit = 5;
		//public static final int kMinorArmCurrentLimit = 8;

		// speed limits for the arms
		public static final double kMajorPIDOutputLimit = 1;
		public static final double kMinorPIDOutputLimit = 1;

		public static final double kMaxMajorVelRadiansPerSec = (Math.PI * 10) * 60;
		public static final double kMaxMajorAccelRadiansPerSec = (Math.PI * 6.25 * 60);

		
		public static final double kMaxMinorVelRadiansPerSec = (Math.PI * 8) * 60;
		public static final double kMaxMinorAccelRadiansPerSec = (Math.PI * 7) * 60;

		// angle limits for the arms
		public static final double kMajorArmConstraints = 110;
		public static final double kMinorArmConstraints = 180;
 
		// Arm PID constants
		public static final PIDGains kMajorArmGains = new PIDGains(0.0035, 0.0000025, 0.002);

		public static final PIDGains kMinorArmGains = new PIDGains(0.002, 0.0000008, 0.001);

		public static enum kArmPoses {
			TUCKED,
			LOW_SCORE,
			MID_SCORE,
			HIGH_SCORE,
			LOW_INTAKE,
			MID_INTAKE,
			HIGH_INTAKE,
			DRIVER_CONTROL,
			KICK_FRONT,
			KICK_BACK,
		}

		public static final HashMap<kArmPoses, double[]> kArmStatesMap = new HashMap<kArmPoses, double[]>() {
			{
				put(kArmPoses.TUCKED, new double[] { 0, 0 });
				put(kArmPoses.LOW_SCORE, new double[] { 0, 80 });
				put(kArmPoses.MID_SCORE, new double[] { 20, 36 });
				put(kArmPoses.HIGH_SCORE, new double[] { 100, 55 });
				put(kArmPoses.LOW_INTAKE, new double[] { -10, 98 });
				put(kArmPoses.MID_INTAKE, new double[] { 13, 33 });
				put(kArmPoses.HIGH_INTAKE, new double[] { 105, 87 });
				put(kArmPoses.DRIVER_CONTROL, new double[] { 0, 0 });
				put(kArmPoses.KICK_FRONT, new double[] { 35, 0 });
				put(kArmPoses.KICK_BACK, new double[] { -35, 0 });
			}
		};

	}

	public static class ClimberConstants {

		public static final int kRightPort = 20;
		public static final int kLeftPort = 21;

		//public static final PIDGains kClimberGains = new PIDGains(0.0035, 0.0000025, 0.002);
		public static final PIDGains kClimberGains = new PIDGains(0.2, 0.0000025, 0.002);
		public static final int kClimberCurrentLimit = 8;
		public static final double kTolerance = .1;

		/**
		 * the total number of motor rotations for one 360 degree rotation of the arm
		 */
		public static final int kClimberTicks = 42 / 5;
		
		public static enum kClimberPoses {
			TUCKED,
			MID,
			HIGH,
		}

		public static final HashMap<kClimberPoses, double[]> kClimberStatesMap = new HashMap<kClimberPoses, double[]>() {
			{
				put(kClimberPoses.TUCKED, new double[] { 1, 0 });
				put(kClimberPoses.HIGH, new double[] { 7, 10 });
			}
		};
	}

	public static class IntakeConstants {

		//
		public static final int kSmartCurrentLimit = 5;
		public static final double kIntakeSpeed = 1.0;
		public static final double kOuttakeSpeed = -1.0;
		public static final double kIntakeOutputCurrentThreshold = 6.0;
		public static final int kIntakeWheelPort = 9;
		//

		//public static final int kRightIntakeWheelPort = 8;
		//public static final int kLeftIntakeWheelPort = 9;

		//public static final int kPnemnaticHubPort = 50;

		//public static final int kCenterDumpSolenoidPort = 0;
		//public static final int kCenterSealerSolenoidPort = 1;

		// NEO Sucker motor CAN ID's
		//public static final int kCenterSuckerPort = 18;

		//public static final int kCenterSuckerCurrentLimit = 6;

		//public static final double kCenterSuckerSetpoint = 0.65;
		//public static final int kHasConeThreshold = 3530;

		public static enum kIntakeStates {
			IDLE,
			INTAKE,
			OUTTAKE,
			DISABLED
		}

		public static final HashMap<kArmPoses, kIntakeStates> kArmStateToIntakeStateMap = new HashMap<kArmPoses, kIntakeStates>() {
			{
				put(kArmPoses.TUCKED, kIntakeStates.INTAKE);
				put(kArmPoses.LOW_SCORE, kIntakeStates.IDLE);
				put(kArmPoses.MID_SCORE, kIntakeStates.IDLE);
				put(kArmPoses.HIGH_SCORE, kIntakeStates.IDLE);
				put(kArmPoses.LOW_INTAKE, kIntakeStates.INTAKE);
				put(kArmPoses.MID_INTAKE, kIntakeStates.INTAKE);
				put(kArmPoses.HIGH_INTAKE, kIntakeStates.INTAKE);
				put(kArmPoses.DRIVER_CONTROL, kIntakeStates.INTAKE);
				put(kArmPoses.KICK_FRONT, kIntakeStates.IDLE);
				put(kArmPoses.KICK_BACK, kIntakeStates.IDLE);
			}
		};

	}

	public static class PhotonVisionConstants {

		public static final boolean VisionEnabled = true;
		public static final boolean PhysicalCamera = false;

		//public static final PoseStrategy poseStrategy = PoseStrategy.AVERAGE_BEST_TARGETS;
		public static final PoseStrategy poseStrategy = PoseStrategy.CLOSEST_TO_REFERENCE_POSE;

		public static double camDiagFOV = 170.0;
		public static double camPitch = 0.0;
		public static double camHeightOffGround = Units.inchesToMeters(6.0);
		// the side to side position of the camera relative to the robot center
		public static double camX = Units.inchesToMeters(-12.0);
		// the front to back position of the camera relative to the robot center
		public static double camY = Units.inchesToMeters(0.0);

		public static Transform3d cameraToRobot = new Transform3d(
                    new Translation3d(
						camX,
						camY,
					 	PhotonVisionConstants.camHeightOffGround
					),
					new Rotation3d(
						0,
						PhotonVisionConstants.camPitch,
						0
					)
				);

		//public static final String CameraName = "Microsoft_LifeCam_HD-3000";
		//public static final String CameraName = "OV5647";
		public static final String CameraName = "cam1";

		// Simulated Vision System.
    	// Configure these to match your PhotonVision Camera,
    	// pipeline, and LED setup.
		public static double sim_camDiagFOV = camDiagFOV; // degrees - assume wide-angle camera
		public static double sim_camPitch = camPitch; // degrees
    	public static double sim_camHeightOffGround = camHeightOffGround; // meters
    	public static double sim_maxLEDRange = 20; // meters
    	public static int sim_camResolutionWidth = 640; // pixels
    	public static int sim_camResolutionHeight = 480; // pixels
    	//public static double sim_minTargetArea = 10; // square pixels
		public static double sim_minTargetArea = 300; // square pixels

		/**
    	* Standard deviations of the vision measurements. Increase these numbers to trust global measurements from vision
    	* less. This matrix is in the form [x, y, theta]ᵀ, with units in meters and radians.
    	*/
		public static double visionMeasurementStdDevsX = 0.5;
		public static double visionMeasurementStdDevsY = 0.5;
		public static double visionMeasurementStdDevsTheta = Units.degreesToRadians(10);
	}

	

	public static final String kRioCANBusName = "rio";

	//public static final String kCanivoreCANBusName = "canivore";

	public static final String logFolders = "/media/sda2/";
	private static final RobotType robot = RobotType.ROBOT_SIMBOT;

	public static RobotType getRobot() {
		if(RobotBase.isReal()) {
			return RobotType.ROBOT_OFFSEASON;
		}

		return RobotType.ROBOT_SIMBOT;
	}

	public static enum RobotType {
		ROBOT_OFFSEASON,
		ROBOT_SIMBOT
	}

	public static enum Mode {
		REAL,
		REPLAY,
		SIM
	}

	public static Mode getMode() {
		switch (getRobot()) {
		  case ROBOT_OFFSEASON:
			return RobotBase.isReal() ? Mode.REAL : Mode.REPLAY;
	
		  case ROBOT_SIMBOT:
			return Mode.SIM;
	
		  default:
			return Mode.REAL;
		}
	  }

	public static enum RobotStatus {
		RobotInit,
		AutoInit,
		AutoPeriodic,
		TelopInit,
		TeleopPeriodic,
		DisabledInit,
		DisabledPeriodic,
	}

	// Autonomous section
	public static Map<String, AutonomousDetail> AutonomousRoutines = Map.of(
		"Test Path 1", 
		new AutonomousDetail(
			"Test Path 1", 
			2.0,
			1.09,
			2.0,
			8.1026 - 1.09,
			.2,
			.2
		)
	);

	public static class ShooterConstants {
		public static final int kSmartCurrentLimit = 5;
		public static final double kSpeed = 1.0;

		public static final int kPrimaryPort = 23;
		public static final int kSecondaryPort = 24;

		public static enum kShooterStates {
			IDLE,
			SHOOT,
			DISABLED
		}
	}
}
