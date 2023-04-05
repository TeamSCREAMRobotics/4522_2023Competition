package frc2023.auto.actions.autonomous;

import frc2023.auto.actions.lib.InstantAction;
import frc2023.subsystems.Intake;
import frc2023.subsystems.Intake.IntakeState;

public class SetIntakeAction extends InstantAction{

    private final IntakeState mIntakeState;
    private final Intake mIntake = Intake.getInstance();

    public SetIntakeAction(IntakeState intakeState){
        mIntakeState = intakeState;
    }

    @Override
    public void start() {
        mIntake.setDesiredState(mIntakeState);
    }

    @Override
    public String getID() {
        return "SetIntakeAction";
    }
}