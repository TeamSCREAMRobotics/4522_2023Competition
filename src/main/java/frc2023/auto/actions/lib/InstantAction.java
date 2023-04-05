package frc2023.auto.actions.lib;

public abstract class InstantAction extends ActionBase {
    
    @Override
    public void run(){}

    @Override
    public void stop(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
}
