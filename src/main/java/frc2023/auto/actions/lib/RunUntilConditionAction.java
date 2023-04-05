package frc2023.auto.actions.lib;

public abstract class RunUntilConditionAction extends ActionBase{
    
    final ParallelAction mActions;
    public RunUntilConditionAction(ActionBase... actions){
        mActions = new ParallelAction(actions);
    }

    @Override
    public void run() {
        mActions.run();
        
    }

    @Override
    public void start() {
        mActions.start();
    }

    @Override
    public void stop(boolean interrupted) {
        mActions.stop(interrupted);
    }

    @Override
    public boolean isFinished() {
        return condition();
    }

    public abstract boolean condition();

    @Override
    public String toString() {
        return "WaitUntilConditionAction: " + mActions.toString();
    }
}
