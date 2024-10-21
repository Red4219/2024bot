package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants.kDriveModes;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveSubsystem;

public class LockWheelsCommand extends Command {

	private static DriveSubsystem _driveSubsystem;
    private static boolean enabledLock = false;

    public LockWheelsCommand(boolean enableLock) {
        _driveSubsystem = RobotContainer.driveSubsystem;
        this.enabledLock = enableLock;
        addRequirements(_driveSubsystem);
    }

    @Override
	public void initialize() {
        // If we are already aiming, set the arm to idle to stop
        /*if(_driveSubsystem.getMode() == kDriveModes.AIM) {
            _driveSubsystem.setMode(kDriveModes.NORMAL);
        } else {
            _driveSubsystem.setMode(kDriveModes.AIM);

        }*/

        if(this.enabledLock == true) {
            _driveSubsystem.setMode(kDriveModes.LOCK_WHEELS);
        } else {
            _driveSubsystem.setMode(kDriveModes.NORMAL);
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
