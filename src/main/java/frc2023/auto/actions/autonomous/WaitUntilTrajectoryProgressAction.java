package frc2023.auto.actions.autonomous;

import frc2023.auto.actions.lib.WaitUntilConditionAction;
import frc2023.subsystems.Swerve;

public class WaitUntilTrajectoryProgressAction extends WaitUntilConditionAction{

    private final Swerve mSwerve = Swerve.getInstance();
    private final double mPercentCompletion;

    public WaitUntilTrajectoryProgressAction(double percentCompletion){
        mPercentCompletion = percentCompletion;
    }

    @Override
    public boolean condition() {
        if(mSwerve.getTrajectoryPercentCompletion().isEmpty()){
            System.out.println(" condition not met because optional is empty.");
            return false;
        } 
        return mSwerve.getTrajectoryPercentCompletion().get() >= mPercentCompletion;
    }

    @Override
    public String getID() {
        return "WaitUntilTrajectoryProgressAction";
    }
}