package frc.robot.commands.Autonomous;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants.kArmPoses;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AutoArmPoseCommand extends Command {

	private static ArmSubsystem armSubsystem;
	private static IntakeSubsystem intakeSubsystem;
	private kArmPoses armPose;

	/** Creates a new ArmPoseCommand. */
	public AutoArmPoseCommand(kArmPoses armPose) {
		armSubsystem = RobotContainer.armSubsystem;
		//intakeSubsystem = RobotContainer.intakeSubsystem;
		this.armPose = armPose;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(armSubsystem);
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {
		//armSubsystem.setSequencedArmState(armPose);
		//intakeSubsystem.updateIntakeFromArmPose(armPose);
		//System.out.println("ArmPoseCommand::initialize() - called");

		//armSubsystem.setSequencedArmState(armPose);
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		armSubsystem.setSequencedArmState(armPose);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		return true;
	}

}

