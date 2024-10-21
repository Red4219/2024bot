package frc.robot.commands.Autonomous;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants.kArmPoses;
import frc.robot.subsystems.ArmSubsystem;

public class AutoArmPoseCommand extends Command {

	private static ArmSubsystem armSubsystem;
	private kArmPoses armPose;

	/** Creates a new ArmPoseCommand. */
	public AutoArmPoseCommand(kArmPoses armPose) {
		if(Constants.kEnableArm) {
			armSubsystem = RobotContainer.armSubsystem;
			this.armPose = armPose;
			addRequirements(armSubsystem);
		}
	}

	// Called when the command is initially scheduled.
	@Override
	public void initialize() {

	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		if(Constants.kEnableArm) {
			armSubsystem.setSequencedArmState(armPose);
		}
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

