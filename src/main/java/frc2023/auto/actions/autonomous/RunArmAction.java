package frc2023.auto.actions.autonomous;

import edu.wpi.first.math.geometry.Translation2d;
import frc2023.auto.actions.lib.ActionBase;
import frc2023.controlboard.ControlBoard;
import frc2023.subsystems.Arm;

public class RunArmAction  extends ActionBase{

    private final Translation2d mTargetPosition;
    private final Arm mArm = Arm.getInstance();
    private final ControlBoard mControlBoard = ControlBoard.getInstance();

    public RunArmAction(Translation2d position){
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
        return false;
    }

    @Override
    public String getID() {
        return "RunArmAction: " + mTargetPosition.toString();
    }
}