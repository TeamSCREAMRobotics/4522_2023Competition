package frc2023.auto.actions.autonomous;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc2023.Constants.SwerveConstants;
import frc2023.auto.actions.lib.ActionBase;
import frc2023.subsystems.Swerve;

public class MoveToPoseAction extends ActionBase{

    private final Pose2d mTargetPose;
    private final Swerve mSwerve = Swerve.getInstance();
    private final double mTranslationTolerance;
    private final Rotation2d mAngleTolerance;
    
    public MoveToPoseAction(Pose2d targetPose, double translationTolerance, Rotation2d angleTolerance){
        mTargetPose = targetPose;
        mTranslationTolerance = translationTolerance;
        mAngleTolerance = angleTolerance;
    }

    public MoveToPoseAction(Pose2d targetPose, boolean precise){
        this(targetPose, (precise? SwerveConstants.positionPreciseTranslationTolerance : SwerveConstants.positionWideTranslationTolerance), (precise? SwerveConstants.positionPreciseAngleTolerance : SwerveConstants.positionWideAngleTolerance));
    }

    public MoveToPoseAction(Translation2d targetPosition, Rotation2d targetRotation, boolean precise){
        this(new Pose2d(targetPosition, targetRotation), precise);
    }

    @Override
    public void start() {
        
    }

    @Override
    public void run() {
        mSwerve.snapToPose(mTargetPose);
    }

    @Override
    public void stop(boolean interrupted) {
        mSwerve.disable();
        mSwerve.temporaryDisableRotation();
    }

    @Override
    public boolean isFinished() {
        return mSwerve.atReference(mTargetPose, mTranslationTolerance, mAngleTolerance);
    }

    @Override
    public String getID() {
        return "Move To Pose Action, " + mTargetPose.toString();
    }
}