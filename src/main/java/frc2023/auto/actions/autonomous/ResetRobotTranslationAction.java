package frc2023.auto.actions.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc2023.auto.actions.lib.InstantAction;
import frc2023.subsystems.Swerve;

public class ResetRobotTranslationAction extends InstantAction{
    
    private final Swerve mSwerve = Swerve.getInstance();
    private final Translation2d mNewTranslation;

    public ResetRobotTranslationAction(Translation2d newTranslation){
        mNewTranslation = newTranslation;
    }

    @Override
    public void start() {
        mSwerve.resetPose(new Pose2d(mNewTranslation, mSwerve.getRobotRotation()));
    }

    @Override
    public String getID() {
        return "ResetRobotTranslationAction";
    }
}
