package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Tools.PhotonVision;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class AimCommand extends Command {

    private static DriveSubsystem _driveSubsystem;
    private static ArmSubsystem _armSubsystem;
    private static ShooterSubsystem _shooterSubsystem;
    private PhotonVision _photonVision;
    private int _targedNumber = 7;

    public AimCommand(PhotonVision photonVision) {

        if(Constants.kEnableArm) {
            _driveSubsystem = RobotContainer.driveSubsystem;
            _armSubsystem = RobotContainer.armSubsystem;
            _shooterSubsystem = RobotContainer.shooterSubsystem;

		    addRequirements(_driveSubsystem, _armSubsystem, _shooterSubsystem);

            _photonVision = photonVision;
        }
    }

    @Override
    public void initialize() {
        if(DriverStation.getAlliance().isPresent()) {
            if(DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
                _targedNumber = 7;
            } else {
                _targedNumber = 4;
            }
        }
    }

    @Override
    public void execute() {
        System.out.println("calling aim command");
    }

    @Override
    public void end(boolean interrupted) {

    }

    public double calculateShooterHeight(double distance) {
        return distance;
    }

    public double calculateShooterSpeed(double distance) {
        return distance;
    }

    @Override
    public boolean isFinished() {

        if(!Constants.kEnableArm) {
            //System.out.println("AimCommand::isFinished() - Constants.kEnableArm is false so returning true");
            return true;
        }

        if(!Constants.kEnablePhotonVision) {
            //System.out.println("AimCommand::isFinished() - Constants.kEnablePhotonVision is false so returning true");
            return true;
        }

        if(_photonVision.canSeeTarget(_targedNumber) == false) {
            System.out.println("AimCommand::isFinished() - cannot see the target so returning true");
            // Stop because we cannot see the target
            return true;
        } 

        double targetYaw = _photonVision.aimAtTarget(_targedNumber);

        if(targetYaw == 0.0) {
            return true;
        }

        if(Math.abs(targetYaw) > Constants.AutoConstants.kAimTargetTolerance) {

            if(targetYaw > 0) {
                _driveSubsystem.drive(0.0, 0.0, -.05);
            } else {
                _driveSubsystem.drive(0.0, 0.0, .05);
            }
            
            return false;
        } 
        
        return true;
    }
    
}
