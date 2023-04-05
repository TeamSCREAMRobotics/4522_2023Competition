package frc2023.auto.actions.lib;

public abstract class WaitUntilConditionAction extends ActionBase {
    

    public WaitUntilConditionAction(){

    }

    @Override
    public void run() {
        
    }

    @Override
    public void start() {

    }

    @Override
    public void stop(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished() {
        return condition();
    }

    public abstract boolean condition();

    @Override
    public String toString() {
        return "WaitUntilConditionAction: " + getID();
    }
}