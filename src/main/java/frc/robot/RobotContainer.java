// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;
import java.util.OptionalLong;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.ClimberConstants.kClimberPoses;
import frc.robot.Constants.IntakeConstants.kIntakeStates;
import frc.robot.Constants.ShooterConstants.kShooterStates;
import frc.robot.Tools.JoystickUtils;
import frc.robot.Tools.PhotonVision;
import frc.robot.Tools.Parts.PathBuilder;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.ArmConstants.kArmPoses;
import frc.robot.commands.ArmAimCommand;
import frc.robot.commands.ArmMoveCommand;
import frc.robot.commands.ArmPoseCommand;
import frc.robot.commands.ChassisAimCommand;
import frc.robot.commands.ClimberPoseCommand;
import frc.robot.commands.FloorIntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.Autonomous.AimCommand;
import frc.robot.commands.Autonomous.AutoArmAimCommand;
import frc.robot.commands.Autonomous.IntakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

	// The robot's subsystems and commands are defined here...
	//public static final DriveSubsystem driveSubsystem = new DriveSubsystem(true);
	public static final DriveSubsystem driveSubsystem = new DriveSubsystem();
	public PhotonVision _photonVision = driveSubsystem.getPhotonVision();
	private static final CommandXboxController operatorController = new CommandXboxController(1);
	public static final ClimberSubsystem climberSubsystem = new ClimberSubsystem(operatorController);
	public static final ArmSubsystem armSubsystem = new ArmSubsystem(driveSubsystem.getPhotonVision());
	public static final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
	public static final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(armSubsystem);
	

	public final static PathBuilder autoBuilder = new PathBuilder();

	//private final CommandJoystick driveJoystick = new CommandJoystick(
			//OperatorConstants.kDriveJoystickPort);
	//private final CommandJoystick turnJoystick = new CommandJoystick(
			//OperatorConstants.kTurnJoystickPort);
	private final CommandXboxController driverController = new CommandXboxController(0);
	//private final XboxController driverController = new XboxController(0);
	//private final XboxController operatorController = new XboxController(1);
	//private final CommandXboxController operatorController = new CommandXboxController(1);
	//private final CommandGenericHID operatorController = new CommandGenericHID(
			//OperatorConstants.kOperatorControllerPort);
	private final CommandXboxController programmerController = new CommandXboxController(
			OperatorConstants.kProgrammerControllerPort);

	private SendableChooser<Command> autoChooser = new SendableChooser<>();
	private Constants.RobotStatus _status = Constants.RobotStatus.RobotInit;

	public SendableChooser<Command> getAutoChooser() {
		return autoChooser;
		//return AutoBuilder.buildAutoChooser();
	}

	Command _autoCommand = null;

	/**
	 * The container for the robot. Contains subsystems, OI devices, and commands.
	 */
	//public RobotContainer(boolean simulation) {
	public RobotContainer() {

		// Configure the trigger bindings
		configureBindings();

		//return AutoBuilder.followPathWithEvents(path);
		//

		NamedCommands.registerCommand("ClimberUp", new ClimberPoseCommand(kClimberPoses.HIGH));
		NamedCommands.registerCommand("ClimberDown", new ClimberPoseCommand(kClimberPoses.TUCKED));
		NamedCommands.registerCommand("Aim", new AimCommand(_photonVision));
		NamedCommands.registerCommand("ArmAimCommand", new AutoArmAimCommand());
		NamedCommands.registerCommand("TimedShootHalfSeconds", new ShootCommand(shooterSubsystem, kShooterStates.SHOOT_SPEAKER, OptionalLong.of(500)));
		NamedCommands.registerCommand("TimedShoot3Seconds", new ShootCommand(shooterSubsystem, kShooterStates.SHOOT_SPEAKER, OptionalLong.of(3000)));
		NamedCommands.registerCommand("TimedIntake2Seconds", new IntakeCommand(kIntakeStates.INTAKE, OptionalLong.of(2000)));
		NamedCommands.registerCommand("TimedIntake1Seconds", new IntakeCommand(kIntakeStates.INTAKE, OptionalLong.of(1000)));
		
		
		try {
			autoChooser.addOption("5 Auto Amp 1", AutoBuilder.buildAuto("5 Auto Amp 1"));
			autoChooser.addOption("5 Auto Amp 2", AutoBuilder.buildAuto("5 Auto Amp 2"));
			autoChooser.addOption("Auto 1", AutoBuilder.buildAuto("Auto 1"));
			autoChooser.addOption("Auto 2", AutoBuilder.buildAuto("Auto 2"));
			autoChooser.addOption("Auto 3", AutoBuilder.buildAuto("Auto 3"));
			autoChooser.addOption("auto-phil", AutoBuilder.buildAuto("auto-phil"));
		} catch (Exception e) {
			//System.out.println(e.getStackTrace());
			String asdf = e.getStackTrace().toString();
			System.out.println("RobotContainer()::RobotContainer() - error: " + e.getMessage());
		}

		//autoChooser = AutoBuilder.buildAutoChooser();

		// region Def Auto
		Shuffleboard.getTab("Autonomous").add("Auto", autoChooser);
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be
	 * created via the {@link Trigger#Trigger(java.util.function.BooleanSupplier)}
	 * constructor with an arbitrary predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
	 * {@link CommandXboxController
	 * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or
	 * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {

		//test
		//driverController.button(1).whileTrue(new RunCommand(() -> driveSubsystem.goToPose()));

		//driverController.button(3).onTrue(new ClimberPoseCommand(kClimberPoses.HIGH));
		//driverController.button(4).onTrue(new ClimberPoseCommand(kClimberPoses.TUCKED));

		/*driverController.button(1).onTrue(
				intakeSubsystem.intakeCommand())
				.onFalse(intakeSubsystem.idleCommand());*/

		/*driverController.button(1).onTrue(
				shooterSubsystem.timedShootCommand(3000))
				.onFalse(shooterSubsystem.idleCommand());*/

		// This was working
		/*
		driverController.rightTrigger().whileTrue(
				new IntakeCommand(kIntakeStates.INTAKE, OptionalLong.empty())
			);*/

		driverController.rightTrigger().onTrue(
			new IntakeCommand(kIntakeStates.INTAKE, OptionalLong.empty())
		);

		driverController.rightTrigger().onFalse(
			new IntakeCommand(kIntakeStates.IDLE, OptionalLong.empty())
		);

		/*driverController.rightTrigger().onFalse(
				//shooterSubsystem.shootCommand()
				//new AimCommand(_photonVision)
				//intakeSubsystem.intakeCommand()
				new IntakeCommand(kIntakeStates.INTAKE, OptionalLong.empty())
			);*/
			//.onFalse(shooterSubsystem.idleCommand());
			//.onFalse(new IntakeCommand(intakeSubsystem, kIntakeStates.IDLE, OptionalLong.empty()));

		/*driverController.button(2).onTrue(
			new IntakeCommand(kIntakeStates.BUMP, OptionalLong.of(500))
		);*/

		/*driverController.button(2).onTrue(
		//driverController.rightTrigger().onTrue(
			//new FloorIntakeCommand(true)
			Commands.parallel(new IntakeCommand(kIntakeStates.INTAKE, OptionalLong.empty()), new ArmPoseCommand(kArmPoses.GROUND_INTAKE))
		);*/

		/*driverController.button(3).onTrue(
			//new FloorIntakeCommand(true)
			//Commands.parallel(new IntakeCommand(kIntakeStates.INTAKE, OptionalLong.empty()), new ArmPoseCommand(kArmPoses.HUMAN_ELEMENT_INTAKE))
			new IntakeCommand(kIntakeStates.INTAKE, OptionalLong.empty())
		);*/

		/*driverController.button(4).onTrue(
			//new FloorIntakeCommand(true)
			//Commands.parallel(new IntakeCommand(kIntakeStates.INTAKE, OptionalLong.empty()), new ArmPoseCommand(kArmPoses.HUMAN_ELEMENT_INTAKE))
			new ClimberPoseCommand(kClimberPoses.HIGH)
		);*/

		/*operatorController.button(1).onTrue(
			//new ArmAimCommand()
			new ArmPoseCommand(kArmPoses.HUMAN_ELEMENT_INTAKE)
		);*/

		operatorController.button(1).onTrue(
			new ArmPoseCommand(kArmPoses.GROUND_INTAKE)
		);

		operatorController.button(2).onTrue(
			new ArmPoseCommand(kArmPoses.SPEAKER_SCORE_POST)
		);

		operatorController.button(3).onTrue(
			new ArmPoseCommand(kArmPoses.AMP_SCORE)
		);

		operatorController.button(4).onTrue(
			new ArmPoseCommand(kArmPoses.SPEAKER_SCORE)
		);

		//operatorController.button(1).onTrue(
		operatorController.rightTrigger().onTrue(
			//new ShootCommand(shooterSubsystem, kShooterStates.SHOOT_SPEAKER, OptionalLong.empty())

			// This will check if the arm is in position to shoot at the amp, then set the shooter accordingly, else shoot speaker
			//new ShootCommand(shooterSubsystem, (armSubsystem.getArmState() == kArmPoses.AMP_SCORE) ? kShooterStates.SHOOT_AMP : kShooterStates.SHOOT_SPEAKER, OptionalLong.empty())
			new ShootCommand(shooterSubsystem, kShooterStates.SHOOT_SPEAKER, OptionalLong.empty())
		);
		
		operatorController.leftTrigger().onTrue(
				//shooterSubsystem.shootCommand()
				//new AimCommand(_photonVision)
				//intakeSubsystem.intakeCommand()
				new IntakeCommand(kIntakeStates.OUTTAKE, OptionalLong.empty())
			);

		operatorController.leftTrigger().onFalse(
				//shooterSubsystem.shootCommand()
				//new AimCommand(_photonVision)
				//intakeSubsystem.intakeCommand()
				new IntakeCommand(kIntakeStates.IDLE, OptionalLong.empty())
			);

		operatorController.button(7).onTrue(
				new IntakeCommand(kIntakeStates.INTAKE_IGNORE_NOTE, OptionalLong.empty())
			);

		// Always point the robot at the target
		//operatorController.button(2).onTrue(
		/*operatorController.leftTrigger().onTrue(
			Commands.parallel(new ChassisAimCommand(), new ArmAimCommand())			
		);*/

		//operatorController.button(3).whileTrue(new RunCommand(() -> driveSubsystem.goToPose(Constants.PoseDefinitions.kFieldPoses.AMPLIFIER)));
		//operatorController.button(4).whileTrue(new RunCommand(() -> driveSubsystem.goToPose(Constants.PoseDefinitions.kFieldPoses.SOURCE)));


		operatorController.rightBumper().whileTrue(new RepeatCommand(new ArmMoveCommand(0.03)));
		operatorController.leftBumper().whileTrue(new RepeatCommand(new ArmMoveCommand(-0.03)));

		//asdf
		//JoystickUtils.processJoystickInput(operatorController.getLeftY());


		// region Arm Commands
		// Schedule ArmPoseCommand when operator presses coresponding button.
		// scoring commands
		/*operatorController.button(1).onTrue(new FloorIntakeCommand(false));
		operatorController.button(2).onTrue(new ArmPoseCommand(kArmPoses.MID_SCORE));
		operatorController.button(3).onTrue(new ArmPoseCommand(kArmPoses.HIGH_SCORE));

		// intaking commands
		operatorController.button(6).onTrue(new FloorIntakeCommand(true));
		operatorController.button(7).onTrue(new ArmPoseCommand(kArmPoses.MID_INTAKE));
		operatorController.button(8).onTrue(new ArmPoseCommand(kArmPoses.HIGH_INTAKE));
		programmerController.b().onTrue(new FloorIntakeCommand(true));

		// tuck arms
		operatorController.button(4).onTrue(new ArmPoseCommand(kArmPoses.TUCKED));
		programmerController.y().onTrue(new ArmPoseCommand(kArmPoses.TUCKED));

		// Switches sides of the robot
		operatorController.button(18).onTrue(new ArmSwitchCommand());*/

		//operatorController.button(11).onTrue(armSubsystem.toggleArmMotors());
		//operatorController.button(13).onTrue(armSubsystem.zeroArms());
		// endregion

		// Sucking is set to be the defaut state of the intake
		/*operatorController.button(5).onTrue(
				intakeSubsystem.outtakeCommand())
				.onFalse(intakeSubsystem.idleCommand());

		operatorController.button(10).onTrue(
				intakeSubsystem.outtakeCommand())
				.onFalse(intakeSubsystem.idleCommand());

		operatorController.button(21).onTrue(intakeSubsystem.disableCommand());*/

		// region Targeting Commmands
		/*driveJoystick.button(3).whileTrue(new LLAlignCommand(false));
		driveJoystick.button(4).whileTrue(new LLAlignCommand(true));
		driveJoystick.button(5).whileTrue(new BalanceCommand());
		driveJoystick.button(6).whileTrue(new LLTargetCubeCommand(1000));
		programmerController.a().whileTrue(new LLAlignCommand(false));
		programmerController.x().whileTrue(new TurnCommand(180));*/
		// endregion

		// programmerController.y().whileTrue(new BalanceCommand()); // TODO: stress
		// test balance
		//operatorController.button(12).whileTrue(new BalanceCommand());

		// region Drive Commands
		//driveJoystick.button(11).onTrue(new InstantCommand(() -> driveSubsystem.zeroHeading()));
		//driveJoystick.button(12).onTrue(driveSubsystem.toggleFieldCentric());

		//programmerController.button(8).onTrue(new InstantCommand(() -> driveSubsystem.zeroHeading()));
		//programmerController.button(6).onTrue(driveSubsystem.toggleFieldCentric());

		//driveJoystick.povUp().whileTrue(
		//		new RunCommand(() -> driveSubsystem.robotCentricDrive(0.05, 0, 0), driveSubsystem));
		//driveJoystick.povDown().whileTrue(
		//		new RunCommand(() -> driveSubsystem.robotCentricDrive(-0.05, 0, 0), driveSubsystem));

		// Swerve Drive method is set as default for drive subsystem
		driveSubsystem.setDefaultCommand(
				/*new RunCommand(
						() -> driveSubsystem.drive(
								-JoystickUtils.processJoystickInput(driveJoystick.getY())
										- JoystickUtils.processJoystickInput(programmerController.getLeftY()), // x axis
								-JoystickUtils.processJoystickInput(driveJoystick.getX())
										- JoystickUtils.processJoystickInput(programmerController.getLeftX()), // y axis
								-JoystickUtils.processJoystickInput(turnJoystick.getX())
										- JoystickUtils.processJoystickInput(programmerController.getRightX()), // rot
																												// axis
								driveJoystick.getHID().getRawButton(1), // turbo boolean
								driveJoystick.getHID().getRawButton(2)), // sneak boolean
						driveSubsystem)*/
				new RunCommand(() -> driveSubsystem.drive(
					//JoystickUtils.processJoystickInput(-driverController.getLeftY()),
					(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? JoystickUtils.processJoystickInput(-driverController.getLeftY()) : JoystickUtils.processJoystickInput(driverController.getLeftY()),
					JoystickUtils.processJoystickInput(driverController.getLeftX()),
					//-JoystickUtils.processJoystickInput((driverController.getHID().isConnected()) ? driverController.getRawAxis(2) : driverController.getRightX()),
					-JoystickUtils.processJoystickInput(driverController.getRightX()),
					//-JoystickUtils.processJoystickInput( driverController.getRawAxis(2) ),
					true,
					false
				),
				driveSubsystem
			)
		);

		// endregion
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {

		//driveSubsystem.setHeading(180);
		//Timer.delay(0.1);
		// An example command will be run in autonomous
		return autoChooser.getSelected();
	}

	public void setStatus(Constants.RobotStatus status) {

		// Check if nothing has changed
		if(_status == status && _autoCommand == autoChooser.getSelected()) {
			return;
		}

		_status = status;
		_autoCommand = autoChooser.getSelected();

		if(status == Constants.RobotStatus.DisabledPeriodic) {
			driveSubsystem.setAutoCommandSelected(autoChooser.getSelected());
		}

		driveSubsystem.setRobotStatus(status);
	}
}