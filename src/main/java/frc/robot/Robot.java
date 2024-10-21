// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
	private Command autonomousCommand;

	private RobotContainer robotContainer;
	private REVPhysicsSim simulator;
	private Constants.RobotStatus robotStatus = Constants.RobotStatus.RobotInit;

	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	@Override
	public void robotInit() {
		// Instantiate our RobotContainer. This will perform all our button bindings,
		// and put our autonomous chooser on the dashboard.

		//Logger logger = Logger.getInstance();

		//logger.recordMetadata("2023_OFFSEASON", "robot"); // Set a metadata value

		/*for (int port = 5800; port <= 5805; port++) {
			PortForwarder.add(port, "limelight.local", port);
		}*/

		// Get the UsbCamera from CameraServer
        //UsbCamera camera = CameraServer.startAutomaticCapture();

		// Set the resolution
		//camera.setResolution(640, 480);

		Pathfinding.setPathfinder(new LocalADStarAK());

		switch (Constants.getMode()) {
			case REAL:
				//Logger.addDataReceiver(new WPILOGWriter(Constants.logFolders));
				//logger.addDataReceiver(new WPILOGWriter(Constants.logFolders));
				Logger.addDataReceiver(new NT4Publisher());
				//logger.addDataReceiver(new NT4Publisher());
				break;
			case SIM:
				Logger.addDataReceiver(new NT4Publisher());
				//logger.addDataReceiver(new NT4Publisher());
				break;
			case REPLAY:
				break;
		}

		//PathPlannerServer.startServer(5811);

		robotContainer = new RobotContainer();
		DriverStation.silenceJoystickConnectionWarning(true);

		//Logger.getInstance().addDataReceiver(new NT4Publisher());

		Logger.start();

		//Logger.getInstance().start(); // Start logging! No more data receivers, replay sources, or metadata values may
                                      // be added.

		robotContainer.setStatus(robotStatus);
	}

	/**
	 * This function is called every 20 ms, no matter the mode. Use this for items
	 * like diagnostics that you want ran during disabled, autonomous, teleoperated
	 * and test.
	 * <p>
	 * This runs after the mode specific periodic functions, but before LiveWindow
	 * and SmartDashboard integrated updating.
	 */
	@Override
	public void robotPeriodic() {
		// Runs the Scheduler. This is responsible for polling buttons, adding
		// newly-scheduled commands, running already-scheduled commands, removing
		// finished or interrupted commands, and running subsystem periodic() methods.
		// This must be called from the robot's periodic block in order for anything in
		// the Command-based framework to work.
		CommandScheduler.getInstance().run();

		/*
		Logger.getInstance().recordOutput("Power/BatteryVoltage", RobotController.getBatteryVoltage());
		//Logger.getInstance().recordOutput("Power/BrownOutVoltage", RobotController.getBrownoutVoltage());
		Logger.getInstance().recordOutput("Power/IsBrownedOut", RobotController.isBrownedOut());
		Logger.getInstance().recordOutput("CAN/ReceiveErrorCount", RobotController.getCANStatus().receiveErrorCount);
		Logger.getInstance().recordOutput("CAN/TransmitErrorCount", RobotController.getCANStatus().transmitErrorCount);
		Logger.getInstance().recordOutput("CAN/PercentBusUtilization", RobotController.getCANStatus().percentBusUtilization);
		*/

		Logger.recordOutput("Power/BatteryVoltage", RobotController.getBatteryVoltage());
		//Logger.getInstance().recordOutput("Power/BrownOutVoltage", RobotController.getBrownoutVoltage());
		Logger.recordOutput("Power/IsBrownedOut", RobotController.isBrownedOut());
		Logger.recordOutput("CAN/ReceiveErrorCount", RobotController.getCANStatus().receiveErrorCount);
		Logger.recordOutput("CAN/TransmitErrorCount", RobotController.getCANStatus().transmitErrorCount);
		Logger.recordOutput("CAN/PercentBusUtilization", RobotController.getCANStatus().percentBusUtilization);
	}

	/** This function is called once each time the robot enters Disabled mode. */
	@Override
	public void disabledInit() {
		robotContainer.setStatus(Constants.RobotStatus.DisabledInit);
	}

	@Override
	public void disabledPeriodic() {
		robotContainer.setStatus(Constants.RobotStatus.DisabledPeriodic);
	}

	/**
	 * This autonomous runs the autonomous command selected by your
	 * {@link RobotContainer} class.
	 */
	@Override
	public void autonomousInit() {

		/*if(DriverStation.getAlliance().isPresent()) {
			System.out.println("it is present");
		} else {
			System.out.println("it is not present");
		}*/

		autonomousCommand = robotContainer.getAutonomousCommand();

		robotContainer.setStatus(Constants.RobotStatus.AutoInit);

		// schedule the autonomous command (example)
		if (autonomousCommand != null) {
			autonomousCommand.schedule();
		}
	}

	/** This function is called periodically during autonomous. */
	@Override
	public void autonomousPeriodic() {
		robotContainer.setStatus(Constants.RobotStatus.AutoPeriodic);
	}

	@Override
	public void teleopInit() {
		// This makes sure that the autonomous stops running when
		// teleop starts running. If you want the autonomous to
		// continue until interrupted by another command, remove
		// this line or comment it out.
		if (autonomousCommand != null) {
			autonomousCommand.cancel();
		}

		robotContainer.setStatus(Constants.RobotStatus.TelopInit);
	}

	/** This function is called periodically during operator control. */
	@Override
	public void teleopPeriodic() {
		robotContainer.setStatus(Constants.RobotStatus.TeleopPeriodic);
	}

	@Override
	public void testInit() {
		// Cancels all running commands at the start of test mode.
		CommandScheduler.getInstance().cancelAll();
	}

	/** This function is called periodically during test mode. */
	@Override
	public void testPeriodic() {
	}

	/** This function is called once when the robot is first started up. */
	@Override
	public void simulationInit() {
		//simulation = true;
		simulator = REVPhysicsSim.getInstance();
		simulator.run();
	}

	/** This function is called periodically whilst in simulation. */
	@Override
	public void simulationPeriodic() {
		REVPhysicsSim.getInstance().run();
	}
}
