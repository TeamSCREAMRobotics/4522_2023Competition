package frc2023.auto.actions.autonomous;

import frc2023.auto.actions.lib.ActionBase;
import frc2023.subsystems.Intake;
import frc2023.subsystems.Intake.IntakeState;

public class RunIntakeAction extends ActionBase{
    
    private final Intake mIntake = Intake.getInstance();
    private IntakeState mIntakeState;

    public RunIntakeAction(IntakeState state){
        mIntakeState = state;
    }

    @Override
    public void start() {
        
    }

    @Override
    public void run() {
        mIntake.setDesiredState(mIntakeState);
        
    }

    @Override
    public void stop(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public String getID() {
        return "RunIntakeAction";
    }
}