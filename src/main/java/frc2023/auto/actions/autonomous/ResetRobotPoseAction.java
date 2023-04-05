package frc2023.auto.actions.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc2023.auto.actions.lib.InstantAction;
import frc2023.subsystems.Swerve;

public class ResetRobotPoseAction extends InstantAction{
    
    private final Swerve mSwerve = Swerve.getInstance();
    private final Pose2d mNewPose;

    public ResetRobotPoseAction(Pose2d newPose){
        mNewPose = newPose;
    }

    public ResetRobotPoseAction(Translation2d translation, Rotation2d rotation){
        mNewPose = new Pose2d(translation, rotation);
    }

    @Override
    public void start() {
        mSwerve.resetPose(mNewPose);
    }

    @Override
    public String getID() {
        return "ResetRobotPoseAction";
    }
}