package frc2023.auto.actions.autonomous;

import edu.wpi.first.wpilibj.Timer;
import frc2023.auto.actions.lib.ActionBase;

public class DelayedAction extends ActionBase{

    private ActionBase mAction;
    private double mTimeDelay;
    private Timer mTimer = new Timer();
    private boolean hasStarted = false;

    public DelayedAction(double timeDelay, ActionBase action){
        mAction = action;
        mTimeDelay = timeDelay;
    }  

    @Override
    public void start() {
        mTimer.reset();
        mTimer.start();
        
    }

    @Override
    public void run() {
        if(mTimer.get() >= mTimeDelay){
            if(!hasStarted){
                mAction.start();
                hasStarted = true;
            } else{
                mAction.run();
            }
        }
        
    }

    @Override
    public void stop(boolean interrupted) {
        mAction.stop(interrupted);
        
    }

    @Override
    public boolean isFinished() {
        return mAction.isFinished() && mTimer.get() >= mTimeDelay;
    }

    @Override
    public String getID() {
        return "Delayed Instant Action: " + mAction.toString();
    }
}
