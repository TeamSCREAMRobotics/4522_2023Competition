package frc2023.auto.actions.autonomous;

import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.Timer;
import frc2023.Constants;
import frc2023.Constants.SwerveConstants;
import frc2023.auto.actions.lib.ActionBase;
import frc2023.subsystems.Swerve;

public class AutoBalanceAction extends ActionBase{
    
    private final Swerve mSwerve = Swerve.getInstance();
    private final PIDController mAutoBalanceController;
    
    private double mLastPitch = 0;
    private double mTimestamp = 0;
    
    public CircularBuffer dPitchBuffer = new CircularBuffer(7);
    public CircularBuffer pitchBuffer = new CircularBuffer(7);
    public final boolean mBackward;

    public AutoBalanceAction(boolean backward){
		mAutoBalanceController = ScreamUtil.createPIDController(SwerveConstants.autoBalancePIDConstants, Constants.kSubsystemPeriodSeconds);
        mBackward = backward;
    }

    @Override
    public void start() {
        
    }

    @Override
    public void run() {
        mSwerve.setSnapAngle((mBackward? SwerveConstants.robotBackwardAngle : SwerveConstants.robotForwardAngle));

        Rotation2d pitch = mSwerve.getPitch();

        double dPitch = (pitch.getDegrees()-mLastPitch) / (Timer.getFPGATimestamp()-mTimestamp);
        dPitchBuffer.addFirst(dPitch);
        pitchBuffer.addFirst(mSwerve.getPitch().getDegrees());

        double bufferedDPitch = ScreamUtil.getCircularBufferAverage(dPitchBuffer);
        double bufferedPitch = ScreamUtil.getCircularBufferAverage(pitchBuffer);

        double driveSpeed = (mBackward? -1: 1) * Math.signum(bufferedPitch)*mAutoBalanceController.calculate(bufferedPitch, 0) * pitch.getSin();
        boolean platformFalling = Math.signum(bufferedDPitch) != Math.signum(bufferedPitch);

        if(platformFalling && Math.abs(bufferedPitch) < SwerveConstants.kPitchToStopSwerveDuringBalance){
            mSwerve.disable();
        } else{
            mSwerve.drive(new Translation2d(0, driveSpeed), 0, false);
        }
    

        mTimestamp = Timer.getFPGATimestamp();
        mLastPitch = mSwerve.getPitch().getDegrees();
    }

    @Override
    public void stop(boolean interrupted) {
        mSwerve.disable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public String getID() {
        return "Auto Balance";
    }
}