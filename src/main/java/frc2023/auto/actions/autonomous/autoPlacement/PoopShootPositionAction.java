package frc2023.auto.actions.autonomous.autoPlacement;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

import frc2023.Constants.SwerveConstants;
import frc2023.Constants.IntakeConstants.ShooterConstants;
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
    private final double mWaitBeforeShootDuration;
    private final Timer mTimerSinceStart = new Timer();

    public PoopShootPositionAction(Translation2d targetPosition, double waitBeforeShootDuration, double shootDuration){
        mTargetPosition = targetPosition;
        mWaitBeforeShootDuration = waitBeforeShootDuration;
        mShootDuration = shootDuration;
    }  

    public PoopShootPositionAction(Translation2d targetPosition, double shootDuration){
        this(targetPosition, 0.0, shootDuration);
    }

    public PoopShootPositionAction(Translation2d targetPosition){
        this(targetPosition, ShooterConstants.kDefaultAutoShootDuration);
    }

    @Override
    public void start() {
        mTimerSinceStart.reset();
        mTimerSinceStart.start();
    }

    @Override
    public void run() {
        mSwerve.snapToPose(new Pose2d(mTargetPosition, SwerveConstants.robotBackwardAngle));
        mArm.setPoopShootPosition();

        if(mTimerSinceStart.get() < mWaitBeforeShootDuration){
            mIntake.retract();
        } else{
            mIntake.poopShootFromChargeLineAuto();
        }

        if(mSwerve.atTranslationReference(mTargetPosition, SwerveConstants.positionPreciseTranslationTolerance)){
            mShootTimer.start();
            mIntake.poopShootFromChargeLineAuto();//forces it to shoot even if the timeBeforeShoot isn't done
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
        return String.join(", ", "PoopShootPositionAction", mTargetPosition.toString(), Double.toString(mWaitBeforeShootDuration), Double.toString(mShootDuration));
    }    
}
