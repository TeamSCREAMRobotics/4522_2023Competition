package frc2023.swerve;

import java.util.Optional;

import com.team254.SwerveSetpoint;
import com.team254.SwerveSetpointGenerator;
import com.team254.SwerveSetpointGenerator.KinematicLimits;
import com.team4522.lib.pid.PIDConstants;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import frc2023.Constants;
import frc2023.Constants.*;
import frc2023.subsystems.Swerve.DodgeDirection;
import frc2023.swerve.SwerveRotationHelper.RotationHelperMode;

public class SwerveDriveHelper {
    
    public SwerveRotationHelper mRotationHelper;
	private final SwerveDriveKinematics kinematics;
    private final Translation2d defaultCenterOfRotation = new Translation2d();
    private Translation2d dodgePivotCenter = new Translation2d();
    private SwerveSetpointGenerator mSetpointGenerator;
    private KinematicLimits mKinematicLimits = SwerveConstants.defaultKinematicsLimits;

    private final PIDController mTrajectoryXController;
    private final PIDController mTrajectoryYController;
    private final PIDController mTrajectoryThetaController;
    
    private final PIDController mPositionXController;
    private final PIDController mPositionYController;
    private final PIDController mPositionThetaController;

    private final PIDController mVisionXController;
    private final PIDController mVisionYController;
    private final PIDController mVisionThetaController;

    private double mTrajectoryXTarget = 0;
    private double mTrajectoryYTarget = 0;
    private Rotation2d mTrajectoryThetaTarget = new Rotation2d();

    enum DriveMode{
        DISABLED, SNAP_POSITION, TRAJECTORY, MANUAL, MANUAL_FACE_POINT, EVASIVE_MANEUVERS, MANUAL_TARGETANGLE, VISION_SNAP_TO_POSITION, VISION_LOCK_X, SNAP_POSITION_FACE_POINT
    }

    private DriveMode mControlMode = DriveMode.DISABLED;

    public SwerveDriveHelper(){
        kinematics = new SwerveDriveKinematics(SwerveConstants.moduleLocations);
        mRotationHelper = new SwerveRotationHelper();
        mSetpointGenerator = new SwerveSetpointGenerator(kinematics, SwerveConstants.moduleLocations);
        
        mPositionXController = ScreamUtil.createPIDController(SwerveConstants.positionXPIDConstants, Constants.kSubsystemPeriodSeconds);
        mPositionXController.disableContinuousInput();
        mPositionYController = ScreamUtil.createPIDController(SwerveConstants.positionYPIDConstants, Constants.kSubsystemPeriodSeconds);
        mPositionYController.disableContinuousInput();
        mPositionThetaController = ScreamUtil.createPIDController(SwerveConstants.positionThetaPIDConstants, Constants.kSubsystemPeriodSeconds);
	 	mPositionThetaController.enableContinuousInput(-Math.PI, Math.PI);

        mTrajectoryXController = ScreamUtil.createPIDController(SwerveConstants.trajectoryTranslationPIDConstants, Constants.kSubsystemPeriodSeconds);
        mTrajectoryXController.disableContinuousInput();
        mTrajectoryYController = ScreamUtil.createPIDController(SwerveConstants.trajectoryTranslationPIDConstants, Constants.kSubsystemPeriodSeconds);
        mTrajectoryYController.disableContinuousInput();
        mTrajectoryThetaController = ScreamUtil.createPIDController(new PIDConstants(), Constants.kSubsystemPeriodSeconds);

        mVisionXController = ScreamUtil.createPIDController(SwerveConstants.visionXPIDConstants, Constants.kSubsystemPeriodSeconds);
        mVisionXController.disableContinuousInput();
        mVisionYController = ScreamUtil.createPIDController(SwerveConstants.visionYPIDConstants,Constants.kSubsystemPeriodSeconds);
        mVisionYController.disableContinuousInput();
        mVisionThetaController = ScreamUtil.createPIDController(SwerveConstants.visionThetaPIDConstants, Constants.kSubsystemPeriodSeconds);
        
	 	mPositionThetaController.enableContinuousInput(-Math.PI, Math.PI);
    }


    //Methods to repeat logic found in multiple drive modes
    private ChassisSpeeds createChassisSpeeds(Translation2d translation, double rotationSpeed, Rotation2d robotRotation, boolean robotCentric){
		if(robotCentric){
            translation = translation.rotateBy(Rotation2d.fromDegrees(-90));//this offset is here because our coordinate system is 90 degrees off from WPI's. For us, 90 degrees is forward because x is to the right.
			return new ChassisSpeeds(translation.getX(), translation.getY(), rotationSpeed);
		}else{
            ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotationSpeed, robotRotation);
			return chassisSpeeds;//these chassisSpeeds are in the wpi coordinate frame because modules zero is forward
		}
	}


    private SwerveModuleState[] chassisSpeedsToSwerveModuleStates(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation){
        SwerveModuleState[] swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds, centerOfRotation);
		SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConstants.kMaxSpeed);
        
		return swerveModuleStates;
	}


    /**
     * This way of updating the chassisSpeeds based on kinematics limits is taken from team 254
     * <p> This is not required for the swerve to function, but it makes the movement slightly more optimized
     */
    private ChassisSpeeds updateDesiredStates(ChassisSpeeds targetChassisSpeeds, Translation2d centerOfRotation) {

        // Set the des_states to account for robot traversing arc.
        Pose2d robot_pose_vel = new Pose2d(targetChassisSpeeds.vxMetersPerSecond * Constants.kSubsystemPeriodSeconds,
                targetChassisSpeeds.vyMetersPerSecond * Constants.kSubsystemPeriodSeconds,
                Rotation2d.fromRadians(targetChassisSpeeds.omegaRadiansPerSecond * Constants.kSubsystemPeriodSeconds));
        Twist2d twist_vel = ScreamUtil.getPoseLog(robot_pose_vel);
        ChassisSpeeds updated_chassis_speeds = new ChassisSpeeds(
                twist_vel.dx / Constants.kSubsystemPeriodSeconds, twist_vel.dy / Constants.kSubsystemPeriodSeconds, twist_vel.dtheta / Constants.kSubsystemPeriodSeconds);

        SwerveSetpoint currentSetpoint = new SwerveSetpoint(targetChassisSpeeds, chassisSpeedsToSwerveModuleStates(targetChassisSpeeds, centerOfRotation));

        return mSetpointGenerator.generateSetpoint(mKinematicLimits, currentSetpoint, updated_chassis_speeds, Constants.kSubsystemPeriodSeconds).chassisSpeeds;
    }


    /**
     * takes chassisSpeeds, optimizes them with the updateDesiredStates() method, and creates SwerveModuleStates based on those chassisSpeeds
     */
    private SwerveModuleState[] generateModuleStatesWithKinematicsLimits(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation){
        return chassisSpeedsToSwerveModuleStates(updateDesiredStates(chassisSpeeds, centerOfRotation), centerOfRotation);
    }


    ///////////////////////////////////////////Swerve Control Modes/////////////////////////////////////////////////////////////////////////////////////////////////////////
    public SwerveModuleState[] getDrive(Translation2d translationInput, double rotationInput, Pose2d robotPose, boolean robotCentric) {
        mControlMode = DriveMode.MANUAL;

        if(Math.abs(rotationInput) > SwerveConstants.kMinAngularSpeed) mRotationHelper.setOpenLoop();
        
        double rotationSpeed = mRotationHelper.calculateRotation(robotPose.getRotation(), rotationInput);
        ChassisSpeeds chassisSpeeds = createChassisSpeeds(translationInput, rotationSpeed, robotPose.getRotation(), robotCentric);
        // System.out.print("   CHASSISPEEDS: " + chassisSpeeds);
        return generateModuleStatesWithKinematicsLimits(chassisSpeeds, defaultCenterOfRotation);
    }


    public SwerveModuleState[] getDriveAndDodge(Translation2d translationInput, DodgeDirection dodgeDirection, Pose2d robotPose) {
        if(mControlMode != DriveMode.EVASIVE_MANEUVERS) {
            if(translationInput.getNorm() > SwerveConstants.kMinSpeed) {
                mControlMode = DriveMode.EVASIVE_MANEUVERS;
                dodgePivotCenter = calculatePivotPoint(translationInput.rotateBy(robotPose.getRotation().times(-1)));
            } else {
                mControlMode = DriveMode.MANUAL;//if we aren't driving the swerve, then stay run in manual mode
                dodgePivotCenter = defaultCenterOfRotation;
            }
        }
        mRotationHelper.setOpenLoop();
        double magnitude = (dodgeDirection == DodgeDirection.Clockwise? -1: 1) * translationInput.getNorm() / SwerveConstants.kMaxDriveSpeed * SwerveConstants.kMaxDriveAngularSpeed;
        double rotationSpeed = mRotationHelper.calculateRotation(robotPose.getRotation(), magnitude);

        ChassisSpeeds chassisSpeeds = createChassisSpeeds(new Translation2d(), rotationSpeed, robotPose.getRotation(), false);
        SwerveModuleState[] states = generateModuleStatesWithKinematicsLimits(chassisSpeeds, dodgePivotCenter);
        return states;
    }



    public SwerveModuleState[] getDriveAndFacePoint(Translation2d translationInput, Pose2d robotPose, Translation2d point, boolean robotCentric, Rotation2d faceForwardDirection) {
        mControlMode = DriveMode.MANUAL_FACE_POINT;

		Rotation2d snapAngle = calculateAngleToFacePoint(ScreamUtil.getTangent(robotPose.getTranslation(), point), faceForwardDirection);
		mRotationHelper.setSnap(robotPose.getRotation(), snapAngle);
		double rotationSpeed = mRotationHelper.calculateRotation(robotPose.getRotation(), 0);
        if(point.getDistance(robotPose.getTranslation()) < SwerveConstants.facePointMinimumDistance) rotationSpeed = 0;//if the robot is basically on the point we are facing, we don't want it to rotate, or it might go crazy

		ChassisSpeeds chassisSpeeds = createChassisSpeeds(translationInput, rotationSpeed, robotPose.getRotation(), robotCentric);
        return generateModuleStatesWithKinematicsLimits(chassisSpeeds, defaultCenterOfRotation);
    }


    public SwerveModuleState[] getDriveAndFaceAngle(Translation2d translationInput, Pose2d robotPose,
        Rotation2d targetAngle, boolean robotCentric) {
        mControlMode = DriveMode.MANUAL_TARGETANGLE;
            
        mRotationHelper.setSnap(robotPose.getRotation(), targetAngle);
        double rotationSpeed = mRotationHelper.calculateRotation(robotPose.getRotation(), 0);
        ChassisSpeeds chassisSpeeds = createChassisSpeeds(translationInput, rotationSpeed, robotPose.getRotation(), robotCentric);
        return generateModuleStatesWithKinematicsLimits(chassisSpeeds, defaultCenterOfRotation);
    }

    

    public SwerveModuleState[] getSnapToPosition(Pose2d robotPose, Pose2d targetPose) {
        mControlMode = DriveMode.SNAP_POSITION;
        mRotationHelper.setOpenLoop();

        double xError = targetPose.getX()-robotPose.getX();
        double yError = targetPose.getY()-robotPose.getY();
        Rotation2d thetaError = targetPose.getRotation().minus(robotPose.getRotation());

        double xFeedback = MathUtil.clamp(mPositionXController.calculate(-xError, 0), -SwerveConstants.maxSpeedDuringPostionMode, SwerveConstants.maxSpeedDuringPostionMode);
        double yFeedback = MathUtil.clamp(mPositionYController.calculate(-yError, 0), -SwerveConstants.maxSpeedDuringPostionMode, SwerveConstants.maxSpeedDuringPostionMode);

        double rotationCommand = mPositionThetaController.calculate(-thetaError.getRadians(), 0); 

        ChassisSpeeds chassisSpeeds = createChassisSpeeds(new Translation2d(xFeedback, yFeedback), rotationCommand, robotPose.getRotation(), false);
        return generateModuleStatesWithKinematicsLimits(chassisSpeeds, defaultCenterOfRotation);
    }


    public SwerveModuleState[] getSnapPositionAndFacePoint(Translation2d targetTranslation, Translation2d point, Pose2d robotPose, Rotation2d faceForwardDirection) {
        mControlMode = DriveMode.SNAP_POSITION_FACE_POINT;

        double xError = targetTranslation.getX()-robotPose.getX();
        double yError = targetTranslation.getY()-robotPose.getY();
        double xFeedback = MathUtil.clamp(mPositionXController.calculate(-xError, 0), -SwerveConstants.maxSpeedDuringPostionMode, SwerveConstants.maxSpeedDuringPostionMode);
        double yFeedback = mPositionYController.calculate(-yError, 0);

        Rotation2d snapAngle = calculateAngleToFacePoint(ScreamUtil.getTangent(robotPose.getTranslation(), point), faceForwardDirection);
        mRotationHelper.setSnap(robotPose.getRotation(), snapAngle);
        double rotationSpeed = mRotationHelper.calculateRotation(robotPose.getRotation(), 0);

        if(point.getDistance(robotPose.getTranslation()) < SwerveConstants.facePointMinimumDistance) rotationSpeed = 0;//if the robot is basically on the point we are facing, we don't want it to rotate, or it might go crazy

        ChassisSpeeds chassisSpeeds = createChassisSpeeds(new Translation2d(xFeedback, yFeedback), rotationSpeed, robotPose.getRotation(), false);
        return generateModuleStatesWithKinematicsLimits(chassisSpeeds, defaultCenterOfRotation);
    }


    public SwerveModuleState[] getVisionSnapToPosition(Pose2d robotPose, Pose2d targetPose){
        if(mControlMode != DriveMode.VISION_SNAP_TO_POSITION){
            mRotationHelper.setOpenLoop();
        } 
        mControlMode = DriveMode.VISION_SNAP_TO_POSITION;

        double xError = targetPose.getX()-robotPose.getX();
        double yError = targetPose.getY()-robotPose.getY();
        Rotation2d thetaError = targetPose.getRotation().minus(robotPose.getRotation());

        double xFeedback = mVisionXController.calculate(-xError, 0);
        double yFeedback = mVisionYController.calculate(-yError, 0);
        double rotationCommand = mVisionThetaController.calculate(-thetaError.getRadians(), 0);

        //if(Math.abs(xError) > SwerveConstants.visionThresholdBeforeMoveY) yFeedback = 0;     //Makes the robot align with the x axis before aligning with the y axis.

        ChassisSpeeds chassisSpeeds = createChassisSpeeds(new Translation2d(xFeedback, yFeedback), rotationCommand, robotPose.getRotation(), false);

        return generateModuleStatesWithKinematicsLimits(chassisSpeeds, defaultCenterOfRotation);
    }


    public SwerveModuleState[] getVisionLockX(double yChassisSpeed, Pose2d targetPose, Pose2d robotPose){

        mControlMode = DriveMode.VISION_LOCK_X;

        double xError = targetPose.getX()-robotPose.getX();
        Rotation2d rotationError = targetPose.getRotation().minus(robotPose.getRotation());
        
        double xFeedback = mVisionXController.calculate(-xError, 0);
        double rotationCommand = mVisionThetaController.calculate(-rotationError.getRadians(), 0);

        ChassisSpeeds chassisSpeeds = createChassisSpeeds(new Translation2d(xFeedback, yChassisSpeed), rotationCommand, robotPose.getRotation(), false);

        return generateModuleStatesWithKinematicsLimits(chassisSpeeds, defaultCenterOfRotation);
	}
   
    
    ////////////////////////////////Trajectory following methods////////////////////////////////////////////////////////////////////////////////////////////////////
	private Trajectory mCurrentTrajectory = null;
	private Optional<Rotation2d> mTrajectoryEndAngle;
	private Optional<Translation2d> mTrajectoryFacePoint;
	private Timer mTrajectoryTimer = new Timer();
	private boolean mTrajectroyInProgress = false;
    private Rotation2d mTrajectoryRobotFaceForwardDirection = SwerveConstants.robotForwardAngle;

    /**
     * We have two options for our angle when following trajectories. This option makes the swerve target the endAngle variable for the whole path.
     */
    public void setTrajectoryWithEndAngle(Trajectory trajectory, Rotation2d endAngle, double thetaKP){
		mCurrentTrajectory = trajectory;
		mTrajectoryEndAngle = Optional.of(endAngle);
        mTrajectoryFacePoint = Optional.empty();
        mTrajectoryThetaController.setP(thetaKP);
	}
    

    /**
     * We have two options for our angle when following trajectories. This option makes the swerve face a point for the whole path.
     */
    public void setTrajectoryWithFacePoint(Trajectory trajectory, Translation2d facePoint, double thetaKP, Rotation2d robotFaceForwardDirection){
		mCurrentTrajectory = trajectory;
		mTrajectoryEndAngle = Optional.empty();
        mTrajectoryFacePoint = Optional.of(facePoint);
        mTrajectoryThetaController.setP(thetaKP);
        mTrajectoryRobotFaceForwardDirection = robotFaceForwardDirection;
	}


	private void startTrajectory(Pose2d robotPose){
		mTrajectoryTimer.reset();
		mTrajectoryTimer.start();
		mTrajectroyInProgress = true;
	}


	public SwerveModuleState[] getFollowTrajectory(Pose2d robotPose){
        mControlMode = DriveMode.TRAJECTORY;
		if(!mTrajectroyInProgress){
            startTrajectory(robotPose);
        } 

        if(mCurrentTrajectory == null) {
            System.out.println("null trajectory");
            return generateModuleStatesWithKinematicsLimits(new ChassisSpeeds(), defaultCenterOfRotation);
        }

		double timestamp = mTrajectoryTimer.get();
		State desiredState = mCurrentTrajectory.sample(timestamp);

        mTrajectoryXTarget = desiredState.poseMeters.getX();
        mTrajectoryYTarget = desiredState.poseMeters.getY();

        double xError = mTrajectoryXTarget - robotPose.getX();
        double yError = mTrajectoryYTarget - robotPose.getY();

        if(mTrajectoryEndAngle.isPresent()){
            mTrajectoryThetaTarget = mTrajectoryEndAngle.get();
        } else /* if(mTrajectoryFacePoint.isPresent()) */{
            mTrajectoryThetaTarget = calculateAngleToFacePoint(ScreamUtil.getTangent(robotPose.getTranslation(), mTrajectoryFacePoint.get()), mTrajectoryRobotFaceForwardDirection);
        }
        Rotation2d thetaError = (mTrajectoryThetaTarget.minus(robotPose.getRotation()));

        double xFF = desiredState.velocityMetersPerSecond * desiredState.poseMeters.getRotation().getCos();
        double yFF = desiredState.velocityMetersPerSecond * desiredState.poseMeters.getRotation().getSin();
            
        double xFeedback = mTrajectoryXController.calculate(-xError, 0);
        double yFeedback = mTrajectoryYController.calculate(-yError, 0);
        double thetaFeedback = mTrajectoryThetaController.calculate(-thetaError.getRadians(), 0);

        if(mTrajectoryFacePoint.isPresent()){
            if(mTrajectoryFacePoint.get().getDistance(robotPose.getTranslation()) < SwerveConstants.facePointMinimumDistance) thetaFeedback = 0.0;//if the robot is basically on the point we are facing, we don't want it to rotate, or it might go crazy
        } 

		ChassisSpeeds chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            xFF + xFeedback, yFF + yFeedback, thetaFeedback, robotPose.getRotation());	

        return generateModuleStatesWithKinematicsLimits(chassisSpeeds, defaultCenterOfRotation);
	}


	public void stopTrajectory(){
		mTrajectoryTimer.stop();
		mTrajectroyInProgress = false;
        mCurrentTrajectory = null;
        System.out.println("STOP TRAJECTORY");
        mRotationHelper.setOpenLoop();
	}


	public boolean hasFinishedTrajectory(Pose2d robotPose){
		if(noTrajectory()) return false;
		return mTrajectoryTimer.hasElapsed(mCurrentTrajectory.getTotalTimeSeconds()) && atTrajectoryReference(robotPose);
    }


    private boolean noTrajectory(){
        return mCurrentTrajectory == null || !mTrajectroyInProgress;
    }


    public boolean atTrajectoryReference(Pose2d robotPose){
        if(noTrajectory()) return false;
        return getTrajectoryError(robotPose.getTranslation()) < SwerveConstants.trajectoryTranslationTolerance && Math.abs(mTrajectoryThetaTarget.getRadians()-robotPose.getRotation().getRadians()) < SwerveConstants.trajectoryAngleTolerance.getRadians();
    }

    
    /**
     * This returns an optional because some of our logic will fail if we don't have a trajectory. If we have no trajectory, we just return empty.
     */
    public Optional<Double> getTrajectoryPercentCompletion() {
		if(noTrajectory()) return Optional.empty();
		return Optional.of(MathUtil.clamp(mTrajectoryTimer.get() / mCurrentTrajectory.getTotalTimeSeconds(), 0, 1.0));
    }


    public double getTrajectoryError(Translation2d robotTranslation) {
        return Math.hypot(mTrajectoryXTarget - robotTranslation.getX(), mTrajectoryYTarget - robotTranslation.getY());
    }


    //Rotation Helper Methods
    public void setSnapAngle(Rotation2d robotRotation, Rotation2d target){
        mRotationHelper.setSnap(robotRotation, target);
    }


    public void setHoldAngle(Rotation2d robotRotation, Rotation2d target){
        mRotationHelper.setHold(robotRotation, target);
    }


    public void temporaryDisableRotation() {
        mRotationHelper.setOpenLoop();
    }


    public void disableRotation(){
        mRotationHelper.disable();
    }


    public RotationHelperMode getRotationMode() {
        return mRotationHelper.getMode();
    }


    ///////////////////////////////////////////// Miscellaneous Methods ////////////////////////////////////////////////////////////////
    /**Calculates the pivot point that the swerve needs to rotate around for dodging moves */
    private Translation2d calculatePivotPoint(Translation2d driveVector){
        return driveVector.times(SwerveConstants.kDodgeArcLength / driveVector.getNorm());
    }


    private Rotation2d calculateAngleToFacePoint(Rotation2d tangent, Rotation2d forwardDirection){
        return tangent.minus(SwerveConstants.robotForwardAngle).plus(forwardDirection);
    }

    
    public SwerveDriveKinematics getKinematics(){
        return kinematics;
    }
}