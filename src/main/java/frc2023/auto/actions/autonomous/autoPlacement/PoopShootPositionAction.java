package frc2023.auto.actions.autonomous.autoPlacement;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

import frc2023.Constants.SwerveConstants;
import frc2023.auto.actions.lib.ActionBase;
import frc2023.subsystems.Arm;
import frc2023.subsystems.Intake;
import frc2023.subsystems.Swerve;

public class PoopShootPositionAction extends ActionBase{

    private final Swerve mSwerve = Swerve.getInstance();
    private final Intake mIntake = Intake.getInstance();
    private final Arm mArm = Arm.getInstance();

    private final Translation2d mTargetPosition;
    private final Timer mShootTimer = new Timer();
    private final double mShootDuration;

    public PoopShootPositionAction(Translation2d targetPosition, double shootDuration){
        mTargetPosition = targetPosition;
        mShootDuration = shootDuration;
    }  

    @Override
    public void start() {
        mShootTimer.stop();
        mShootTimer.reset();
    }

    @Override
    public void run() {
        mSwerve.snapToPose(new Pose2d(mTargetPosition, SwerveConstants.robotBackwardAngle));
        mArm.setPoopShootPosition();
        mIntake.poopShootFromChargeLineAuto();
        if(mSwerve.atTranslationReference(mTargetPosition, SwerveConstants.positionPreciseTranslationTolerance)){
            mShootTimer.start();
        } else{
            
        }
    }

    @Override
    public void stop(boolean interrupted) {
        mSwerve.disable();
        mIntake.disable();
        mArm.disable();
    }

    @Override
    public boolean isFinished() {
        return mShootTimer.get() >= mShootDuration;
    }

    @Override
    public String getID() {
        return String.join(", ", "PoopShootPositionAction", mTargetPosition.toString(), Double.toString(mShootDuration));
    }    
}
