package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ArmSubsystem;

public class ArmMoveCommand extends InstantCommand {
    private static ArmSubsystem armSubsystem;
    private double moveAmount = 0.0;

    public ArmMoveCommand(double moveAmount) {
        armSubsystem = RobotContainer.armSubsystem;
        this.moveAmount = moveAmount;

        addRequirements(armSubsystem);
    }

    @Override
	public void initialize() {

    }

    @Override
    public void execute() {
        armSubsystem.moveToPosition(armSubsystem.getPosition() + this.moveAmount);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
