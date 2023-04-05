package frc2023.auto.actions.autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import frc2023.auto.actions.lib.ActionBase;
import frc2023.controlboard.ControlBoard;
import frc2023.subsystems.Arm;

public class ArmPositionAction  extends ActionBase{

    private final Translation2d mTargetPosition;
    private final Arm mArm = Arm.getInstance();
    private final ControlBoard mControlBoard = ControlBoard.getInstance();

    public ArmPositionAction(Translation2d position){
        mTargetPosition = position;
    }

    @Override
    public void start() {

    }

    @Override
    public void run() {
        if(mControlBoard.getArmManualOverride()){
            mArm.disable();
        } else{            
            mArm.setPosition(mTargetPosition, false);   
        }
    }

    @Override
    public void stop(boolean interrupted) {
        mArm.disable();
    }

    @Override
    public boolean isFinished() {
        return mArm.atTargetPosition(mTargetPosition);
    }

    @Override
    public String getID() {
        return "Arm Position Action: " + mTargetPosition.toString();
    }
}