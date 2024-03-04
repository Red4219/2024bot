package frc.robot.commands.Autonomous;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants.kArmPoses;
import frc.robot.subsystems.ArmSubsystem;

public class AutoArmAimCommand extends Command {
    private static ArmSubsystem _armSubsystem;

    @Override
    public void initialize() {
        _armSubsystem = RobotContainer.armSubsystem;
        addRequirements(_armSubsystem);
    }

    @Override
    public void execute() {
        _armSubsystem.setTargetArmState(kArmPoses.AIM);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        if(_armSubsystem.atSetPoint()) {
            return true;
        }

        System.out.println("not at setpoint yet");

        return false;
    }
}
