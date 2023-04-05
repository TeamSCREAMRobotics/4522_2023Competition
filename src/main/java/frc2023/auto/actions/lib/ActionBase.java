package frc2023.auto.actions.lib;

public abstract class ActionBase{
    
    public abstract void start();

    public abstract void run();

    public abstract void stop(boolean interrupted);

    public abstract boolean isFinished();

    public abstract String getID();
    
    @Override
    public String toString() {
        return getID();
    }

    public boolean equals(ActionBase other) {
        return this.toString() == other.toString();
    }
}