package frc2023.auto.actions.lib;

public class EmptyAction extends ActionBase {

    @Override
    public void run() {}

    @Override
    public void start() {}

    @Override
    public void stop(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return true;
    }
    
    @Override
    public String getID() {
        return "Empty Action";
    }
}
