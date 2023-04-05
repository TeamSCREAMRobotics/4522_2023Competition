package frc2023.auto.actions.lib;

import edu.wpi.first.wpilibj.Timer;

public class DelayedInstantAction extends ActionBase{
    
    private final double mDelayDuration;
    private final Timer mTimer = new Timer();
    private final InstantAction[] actions;

    public DelayedInstantAction(double delayDuration, InstantAction... instantActions){
        mDelayDuration = delayDuration;
        actions = instantActions;
    }

    @Override
    public boolean isFinished() {
        return mTimer.get() >= mDelayDuration;
    }

    @Override
    public void run() {
        
    }

    @Override
    public void start() {
        mTimer.reset();
        mTimer.start();
    }

    @Override
    public void stop(boolean interrupted) {
        for(InstantAction a : actions) a.start();
        mTimer.stop();
    }

    @Override
    public String getID() {
        return "DelayedInstantAction, " + actions.toString();
    }
}
