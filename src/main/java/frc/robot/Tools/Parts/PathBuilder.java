// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Tools.Parts;

import java.io.File;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;

/*import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;*/

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.AutoConstants.PathPLannerConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

/** Add your docs here. */
public class PathBuilder {

	private static DriveSubsystem driveSubsystem;
	//private static SwerveAutoBuilder autoBuilder;

	//private static HashMap<String, Command> pathMap = new HashMap<>();
	private static HashMap<String, List<PathPlannerPath>> pathMap = new HashMap<>();

	public PathBuilder() {
		driveSubsystem = RobotContainer.driveSubsystem;

		/*autoBuilder = new SwerveAutoBuilder(
				//driveSubsystem::getPose,
				driveSubsystem::getPoseEstimatorPose2d,
				driveSubsystem::resetOdometry,
				DriveConstants.kDriveKinematics,
				new PIDConstants(
						PathPLannerConstants.kPPDriveGains.kP,
						PathPLannerConstants.kPPDriveGains.kI,
						PathPLannerConstants.kPPDriveGains.kD),
				new PIDConstants(
						PathPLannerConstants.kPPTurnGains.kP,
						PathPLannerConstants.kPPTurnGains.kI,
						PathPLannerConstants.kPPTurnGains.kD),
				driveSubsystem::setModuleStates,
				PathPLannerConstants.kPPEventMap,
				true,
				driveSubsystem);*/

				AutoBuilder.configureHolonomic(
                	driveSubsystem::getPoseEstimatorPose2d, // Robot pose supplier
                	driveSubsystem::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
                	driveSubsystem::getChassisSpeedsRobotRelative, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                	driveSubsystem::setChassisSpeedsRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                	new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                        new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                        4.5, // Max module speed, in m/s
                        0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                        new ReplanningConfig() // Default path replanning config. See the API for the options here
                	),
                	() -> {
                    	// Boolean supplier that controls when the path will be mirrored for the red alliance
                    	// This will flip the path being followed to the red side of the field.
                    	// THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                    	var alliance = DriverStation.getAlliance();
                    	if (alliance.isPresent()) {
	                        return alliance.get() == DriverStation.Alliance.Red;
                    	}
                    	return false;
                	},
                	driveSubsystem // Reference to this subsystem to set requirements
        	);
	}

	//public Command getPathCommand(String path) {
	public List<PathPlannerPath> getPathCommand(String path) {
		return pathMap.get(path);
	}

	public void addPath(String pathName) {
		/*pathMap.put(
					pathName,
					autoBuilder.fullAuto(PathPlanner.loadPathGroup(
							pathName,
							new PathConstraints(
									PathPLannerConstants.kPPMaxVelocity,
									PathPLannerConstants.kPPMaxAcceleration))));*/

		// Use the PathPlannerAuto class to get a path group from an auto
		//List<PathPlannerPath> pathGroup = PathPlannerAuto.getPathGroupFromAutoFile(pathName);
		pathMap.put(pathName, PathPlannerAuto.getPathGroupFromAutoFile(pathName));

		// You can also get the starting pose from the auto. Only call this if the auto actually has a starting pose.
		Pose2d startingPose = PathPlannerAuto.getStaringPoseFromAutoFile(pathName);
	}

	/** fills the path map with paths from the pathplanner directory */
	/*public void populatePathMap() {

		File pathsFolder = new File(Filesystem.getDeployDirectory(), "pathplanner/");
		File[] pathsFilesList = pathsFolder.listFiles();

		for (int i = 0; i < pathsFilesList.length; i++) {

			String pathName = pathsFilesList[i].getName();
			pathName = pathName.substring(0, pathName.length() - 5);

			pathMap.put(
					pathName,
					autoBuilder.fullAuto(PathPlanner.loadPathGroup(
							pathName,
							new PathConstraints(
									PathPLannerConstants.kPPMaxVelocity,
									PathPLannerConstants.kPPMaxAcceleration))));
		}
	}*/

}
