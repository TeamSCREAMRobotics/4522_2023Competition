package frc2023.auto.actions.autonomous;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import frc2023.Constants.SwerveConstants;
import frc2023.auto.actions.lib.ActionBase;
import frc2023.subsystems.Swerve;
/**
 * An action that makes the swerve follow a trajectory. There are two options for this action: making the swerve target an angle and 
 *       using PID to reach the angle throughout the trajectory, and making the swerve face a point for the entirety of the trajectory
 */
public class FollowTrajectoryAction extends ActionBase {

    private final Swerve mSwerve = Swerve.getInstance();
    private final Trajectory mTrajectory;
    private final Optional<Rotation2d> mEndAngle;
    private final Double mThetaKP;
    private final double mTimeout;
    private final Timer mTimer = new Timer();
    private final Rotation2d mRobotForwardAngle;

    public FollowTrajectoryAction(Trajectory trajectory, Rotation2d endAngle, double thetaKP ){
        this(trajectory, endAngle, thetaKP, SwerveConstants.defaultTrajectoryTimeoutSeconds);
    }

    public FollowTrajectoryAction(Trajectory trajectory, Rotation2d endAngle, double thetaKP, double timeoutSeconds){
        mTrajectory = trajectory;
        mEndAngle = Optional.of(endAngle);
        mThetaKP = thetaKP;
        mTimeout = timeoutSeconds + trajectory.getTotalTimeSeconds();
        mRobotForwardAngle = SwerveConstants.robotForwardAngle;
    }



    @Override
    public void run() {
        mSwerve.executeTrajectory();
    }

    @Override
    public void start() {
        
        mSwerve.setTrajectory(mTrajectory, mEndAngle.get(), mThetaKP);
        
        mTimer.reset();
        mTimer.start();
    }

    @Override
    public void stop(boolean interrupted) {
        mSwerve.stopTrajectory();
    }

    @Override
    public boolean isFinished() {
        System.out.println("trajectory finished: " + mSwerve.trajectoryFinished());
        return mSwerve.trajectoryFinished() || mTimer.get() > mTimeout;
    }
    @Override
    public String getID() {
        return String.join(", ", "Follow TrajectoryAction", mTrajectory.toString(), mThetaKP.toString(), mEndAngle.toString());
    }
}