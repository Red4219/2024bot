package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants.kShooterStates;
import frc.robot.subsystems.ShooterSubsystem;
import java.util.OptionalLong;
import java.util.Timer;
import java.util.TimerTask;

public class ShootCommand extends Command {
    private ShooterSubsystem shooterSubsystem;
    private OptionalLong shootTime = OptionalLong.empty();
    private boolean finished = false;
    private kShooterStates shooterState;

    public ShootCommand(ShooterSubsystem shooterSubsystem, kShooterStates state, OptionalLong shootTime) {
        this.shooterSubsystem = shooterSubsystem;
        this.shootTime = shootTime;
        this.shooterState = state;

        if(Constants.kEnableShooter) {
            addRequirements(shooterSubsystem);
        }
    }

    @Override
    public void initialize() {
        if (Constants.kEnableShooter) {
            shooterSubsystem.setShooterState(shooterState);

            if (shootTime.isPresent()) {

                TimerTask task = new TimerTask() {
                    public void run() {
                        System.out.println("stopping the shooter");
                        shooterSubsystem.setShooterState(Constants.ShooterConstants.kShooterStates.STOPPED);
                        finished = true;
                    }
                };
                Timer timer = new Timer("Timer");

                timer.schedule(task, shootTime.getAsLong());
                System.out.println("starting the shooter with initialize");
                finished = false;
            } else {
                finished = true;
            }
        }
    }

    @Override
    public void execute() {
        //System.out.println("execute() called");
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        if (!Constants.kEnableShooter) {
            return true;
        }
        return finished;
    }
}
