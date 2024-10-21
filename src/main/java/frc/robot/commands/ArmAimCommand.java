package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants.kArmPoses;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// This command will set the arm to auto adjust based on the distance
// of the speaker.  If the command is called againl it stops

public class ArmAimCommand extends Command {
    private static ArmSubsystem _armSubsystem;
    private static ShooterSubsystem _shooterSubsystem;

    public ArmAimCommand() {
        if(Constants.kEnableArm) {
            _armSubsystem = RobotContainer.armSubsystem;
            _shooterSubsystem = RobotContainer.shooterSubsystem;
            addRequirements(_armSubsystem, _shooterSubsystem);
        }
    }

    @Override
	public void initialize() {

        if(Constants.kEnableArm) {
            // If we are already aiming, set the arm to idle to stop
            if(_armSubsystem.getArmState() == kArmPoses.AIM) {
                _armSubsystem.setTargetArmState(kArmPoses.IDLE);

                //System.out.println("ArmAimCommand::initialize() - turning off auto aim");
            } else {
                _armSubsystem.setTargetArmState(kArmPoses.AIM);

            }
        }
    }

    @Override
	public void execute() {
        
	}

    @Override
	public void end(boolean interrupted) {
	}

    @Override
	public boolean isFinished() {
        return true;
	}
}
