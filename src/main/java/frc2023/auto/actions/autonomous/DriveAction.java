package frc2023.auto.actions.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc2023.auto.actions.lib.ActionBase;
import frc2023.subsystems.Swerve;

public class DriveAction extends ActionBase{

    private final Swerve mSwerve = Swerve.getInstance();
    private final Translation2d mTranslationInput;
    private final Rotation2d mRotationInput;
    private final boolean mRobotCentric;
    private final Timer mTimer = new Timer();
    private final double mDuration;
    
    public DriveAction(Translation2d translationInput, Rotation2d rotationInput, boolean robotCentric, double duration){
        mTranslationInput = translationInput;
        mRotationInput = rotationInput;
        mRobotCentric = robotCentric;   
        mDuration = duration; 
    }

    public DriveAction(Translation2d translationInput, double rotationInput, boolean robotCentric, double duration){
       this(translationInput, Rotation2d.fromRadians(rotationInput), robotCentric, duration);
    }

    @Override
    public void start() {
        mTimer.reset();
        mTimer.start();
    }

    @Override
    public void run() {
        mSwerve.drive(mTranslationInput, mRotationInput.getRadians(), mRobotCentric);
    }

    @Override
    public void stop(boolean interrupted) {
        mSwerve.disable();
    }

    @Override
    public boolean isFinished() {
        return mTimer.get() >= mDuration;
    }

    @Override
    public String getID() {
        return String.join(", ", "driveAction", mTranslationInput.toString(), mRotationInput.toString(), Boolean.toString(mRobotCentric), Double.toString(mDuration));
    }
    
}
