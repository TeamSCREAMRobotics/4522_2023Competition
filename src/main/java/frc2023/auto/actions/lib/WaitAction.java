package frc2023.auto.actions.lib;

import edu.wpi.first.wpilibj.Timer;

public class WaitAction extends ActionBase {
    
    private final Timer mTimer;
    private final double mDuration;

    public WaitAction(double duration){
        mTimer = new Timer();
        mDuration = duration;
    }

    @Override
    public void run() {}

    @Override
    public void start() {
        mTimer.reset();
        mTimer.start();
    }

    @Override
    public void stop(boolean interrupted) {
        mTimer.stop();
    }

    @Override
    public boolean isFinished() {
        return mTimer.get() >= mDuration;
    }

    @Override
    public String getID() {
        return "WaitAction: duration: " + mDuration;
    }
}
