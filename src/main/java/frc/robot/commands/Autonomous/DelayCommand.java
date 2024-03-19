package frc.robot.commands.Autonomous;

import java.util.OptionalLong;
import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj2.command.Command;

public class DelayCommand extends Command {

    private boolean finished = false;
    private OptionalLong delayTime = OptionalLong.empty();

    public DelayCommand(OptionalLong delayTime) {
        this.delayTime = delayTime;
    }

    @Override
	public void initialize() {
        if(delayTime.isPresent()) {

            TimerTask task = new TimerTask() {
                public void run() {
                    System.out.println("stopping the delay");
                }
            };
            Timer timer = new Timer("Timer");
    
            timer.schedule(task, delayTime.getAsLong());
            System.out.println("starting the delay");
            finished = false;
        } else {
            finished = true;
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
        return finished;
    }
    
}
