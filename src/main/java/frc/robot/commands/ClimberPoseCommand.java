package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ClimberConstants.kClimberPoses;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberPoseCommand extends Command {
	private static ClimberSubsystem climberSubsystem;
	private kClimberPoses climberPose;

	public ClimberPoseCommand(kClimberPoses climberPose) {
		climberSubsystem = RobotContainer.climberSubsystem;
		this.climberPose = climberPose;
		// Use addRequirements() here to declare subsystem dependencies.
		addRequirements(climberSubsystem);
	}

	@Override
	public void initialize() {
		//climberSubsystem.setSequencedClimberState(climberPose);
		//climberSubsystem.setTargetClimberState(climberPose);
		System.out.println("ClimberPoseCommand::initialize() called");
	}

	// Called every time the scheduler runs while the command is scheduled.
	@Override
	public void execute() {
		//System.out.println("ClimberPoseCommand::execute() called");
		climberSubsystem.setTargetClimberState(climberPose);
	}

	// Called once the command ends or is interrupted.
	@Override
	public void end(boolean interrupted) {
		System.out.println("ClimberPoseCommand::end() called");
	}

	// Returns true when the command should end.
	@Override
	public boolean isFinished() {
		//return armSubsystem.getAtTarget(8);
		//return true;

		/*switch(climberPose) {
			case HIGH:
				if(climberSubsystem.isFullyExtended()) return true;
				break;
			case MID:
				return false;
			case TUCKED:
				if(climberSubsystem.isFullyRetracted()) return true;
				break;
			default:
				return false;
		}
		return climberSubsystem.isFullyExtended();*/

		/*if(climberSubsystem.isAtSetPointRight() && climberSubsystem.isAtSetPointLeft()) {
			System.out.println("isFinished returning true");
		}
		
		return climberSubsystem.isAtSetPointRight() && climberSubsystem.isAtSetPointLeft();*/

		return true;
	}
}
