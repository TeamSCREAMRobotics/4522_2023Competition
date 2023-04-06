package frc2023.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.Pigeon2;
import com.team4522.lib.pid.PIDConstants;
import com.team6328.PoseEstimator;
import com.team6328.PoseEstimator.TimestampedVisionUpdate;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc2023.Constants.*;
import frc2023.swerve.SwerveDriveHelper;
import frc2023.swerve.SwerveModule;
import frc2023.swerve.SwerveRotationHelper;

public class Swerve extends Subsystem{//this is the wrapper for a facade design pattern. All of the math for Swerve is in SwerveDriveHelper.
	public final PeriodicIO mPeriodicIO = new PeriodicIO();

	private final Pigeon2 mGyro;
	private final SwerveModule[] mModules;
	public final SwerveDriveHelper mSwerveDriveHelper;
	private final PoseEstimator mPoseEstimator;
	private final Devices mDevices  = Devices.getInstance(); 

	private static Swerve mInstance = null;
	public static Swerve getInstance(){
		if(mInstance == null){
			mInstance = new Swerve();
		}
		return mInstance;
	}

	private Swerve(){
		//Devices
		mGyro = mDevices.dGyro;
		mGyro.setYaw(0);

		mModules = new SwerveModule[]{
			new SwerveModule(mDevices.dFLDrive, mDevices.dFLSteer, mDevices.dFLCancoder, SwerveConstants.FLConstants),
			new SwerveModule(mDevices.dBLDrive, mDevices.dBLSteer, mDevices.dBLCancoder, SwerveConstants.BLConstants),
			new SwerveModule(mDevices.dBRDrive, mDevices.dBRSteer, mDevices.dBRCancoder, SwerveConstants.BRConstants),
			new SwerveModule(mDevices.dFRDrive, mDevices.dFRSteer, mDevices.dFRCancoder, SwerveConstants.FRConstants)
		};
		mSwerveDriveHelper = new SwerveDriveHelper();		
		mPoseEstimator = new PoseEstimator(SwerveConstants.stateStandardDeviations);

		resetPose(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
	}


	public static enum SwerveState{
		DISABLED, FOLLOW_TRAJECTORY, POSITION, VISION, VISION_LOCK_X, DRIVE, DRIVE_WITH_DODGE, DRIVE_FACE_POINT, DRIVE_FACE_ANGLE, LOCK_WHEELS, SNAP_TRANSLATION_AND_FACE_POINT
	}


	public static enum DodgeDirection{
		Clockwise, Counterclockwise
	}


	public static class PeriodicIO{
		//Inputs
		public SwerveState state = SwerveState.DISABLED;
		public Translation2d translationInput = new Translation2d();
		public double rotationInput = 0.0;
		public boolean robotCentric = false;

		public Rotation2d targetAngle = new Rotation2d();
		public Translation2d targetFacePoint = new Translation2d();
		public Pose2d targetPose = new Pose2d();//all of these different position options are redundant, but it doesn't matter because of the design pattern
		public Translation2d targetTranslation = new Translation2d();
		public DodgeDirection dodgeDirection = DodgeDirection.Counterclockwise;
		public Rotation2d facePointForwardDirection = SwerveConstants.robotForwardAngle;

		//Measured values
		public SwerveState lastState = SwerveState.DISABLED;
		public Pose2d lastPose = new Pose2d();
		public Rotation2d lastGyroYaw = new Rotation2d();
		public double[] lastModulePositionsMeters = new double[] {0.0, 0.0, 0.0, 0.0};
		public double timeStamp = 0.0;
		public double lastTimeStamp = 0.0;
		public double dt = 0.0;
		public List<TimestampedVisionUpdate> storedVisionMeasurements = new ArrayList<>();
	}


	////////////////////  Odometry Methods  /////////////////////////////////////////////////////////////	
	public synchronized void updateOdometry(){
		SwerveModulePosition[] wheelDeltas = new SwerveModulePosition[4];
		for (int i = 0; i < 4; i++) {
		  	wheelDeltas[i] = new SwerveModulePosition(
				  (mModules[i].getDistance() - mPeriodicIO.lastModulePositionsMeters[i]),
				  mModules[i].getAbsoluteAngle());
			mPeriodicIO.lastModulePositionsMeters[i] = mModules[i].getDistance();
		}

		Twist2d twist = mSwerveDriveHelper.getKinematics().toTwist2d(wheelDeltas);

		Rotation2d gyroYaw = getGyroYaw();
		Rotation3d robotRotation = getRobotRotation3d();//TODO test the pitch/roll math

		twist = new Twist2d(twist.dx * Math.cos(robotRotation.getY()), twist.dy * Math.cos(robotRotation.getX()), gyroYaw.minus(mPeriodicIO.lastGyroYaw).getRadians());

		mPeriodicIO.lastGyroYaw = gyroYaw;

		 updateVisionData();
		 mPoseEstimator.addDriveData(Timer.getFPGATimestamp(), twist);
	}


    public synchronized void addVisionMeasurement(TimestampedVisionUpdate measurement){//this adds a vision measurement to a list, but does not actually use the vision measurements to update the pose
        mPeriodicIO.storedVisionMeasurements.add(measurement);
    }  
	

    private synchronized void updateVisionData(){//this is where the vision updates list is used to update the robot pose.
        mPoseEstimator.addVisionData(mPeriodicIO.storedVisionMeasurements);
        mPeriodicIO.storedVisionMeasurements.clear();
    }


	public synchronized void resetPose(Pose2d newPose){
		mPoseEstimator.resetPose(newPose);
        System.out.println(" NEW POSE: " + newPose);
		mSwerveDriveHelper.disableRotation();
	}


	public void resetRotation(Rotation2d rotation){
		Pose2d robotPose = getRobotPose();
		resetPose(new Pose2d(robotPose.getTranslation(), rotation));
	}


	public void resetTranslation(Translation2d translation){
		resetPose(new Pose2d(translation, getRobotRotation()));
	}


////////////////  Drive Methods  ////////////////////////////////////////////////////////////////////////
	public void drive(Translation2d translationInput, double rotationInput, boolean robotCentric){
		mPeriodicIO.state = SwerveState.DRIVE;
		mPeriodicIO.translationInput = translationInput;
		mPeriodicIO.rotationInput = rotationInput;
		mPeriodicIO.robotCentric = robotCentric;
	}


	public void lockWheels(){
		mPeriodicIO.state = SwerveState.LOCK_WHEELS;
	}


	public void driveAndDodge(Translation2d translationInput, DodgeDirection dodgeDirection){
		mPeriodicIO.state = SwerveState.DRIVE_WITH_DODGE;
		mPeriodicIO.translationInput = translationInput;
		mPeriodicIO.dodgeDirection = dodgeDirection;
	}


	public void driveAndFacePoint(Translation2d translationInput, Translation2d point, boolean robotCentric, Rotation2d robotFaceForwardDirection){
		mPeriodicIO.state = SwerveState.DRIVE_FACE_POINT;
		mPeriodicIO.translationInput = translationInput;
		mPeriodicIO.targetFacePoint = point;
		mPeriodicIO.robotCentric = robotCentric;
		mPeriodicIO.facePointForwardDirection = robotFaceForwardDirection;
	}


	public void snapTranslationAndFacePoint(Translation2d targetTranslation, Translation2d point, Rotation2d robotfaceForwardDirection){
		mPeriodicIO.state = SwerveState.SNAP_TRANSLATION_AND_FACE_POINT;
		mPeriodicIO.targetTranslation = targetTranslation;
		mPeriodicIO.targetFacePoint = point;
		mPeriodicIO.facePointForwardDirection = robotfaceForwardDirection;
	}


	public void driveAndFaceAngle(Translation2d translationInput, Rotation2d angle, boolean robotCentric){//FIXME this method faces the robot to 0 degrees when there is no input while it should just not turn at all
		mPeriodicIO.state = SwerveState.DRIVE_FACE_ANGLE;
		mPeriodicIO.translationInput = translationInput;
		mPeriodicIO.targetAngle = angle;
		mPeriodicIO.robotCentric = robotCentric;
	}


	public void snapToPose(Pose2d desiredLocation){
		mPeriodicIO.state = SwerveState.POSITION;
		mPeriodicIO.targetPose = desiredLocation;
	}


	public void setVisionSnap(Pose2d targetPose){
		mPeriodicIO.state = SwerveState.VISION;
		mPeriodicIO.targetPose = targetPose;
	}


	public void setVisionLockX(double yChassisSpeed, Pose2d targetPose){
		mPeriodicIO.state = SwerveState.VISION_LOCK_X;
		mPeriodicIO.targetPose = targetPose;
		mPeriodicIO.translationInput = new Translation2d(0, yChassisSpeed);
	}

///////////////////////////////////////  Trajectory  /////////////////////////////////////////////////////////////////

	public void setTrajectoryWithEndAngle(Trajectory trajectory, Rotation2d endAngle, double thetaKP){
		mSwerveDriveHelper.setTrajectoryWithEndAngle(trajectory, endAngle, thetaKP);
	}


	public void setTrajectoryWithFacePoint(Trajectory trajectory, Translation2d facePoint, double thetaKP, Rotation2d robotFaceForwardDirection){
		mSwerveDriveHelper.setTrajectoryWithFacePoint(trajectory, facePoint, thetaKP, robotFaceForwardDirection);
	}


	public void executeTrajectory(){
		mPeriodicIO.state = SwerveState.FOLLOW_TRAJECTORY;
	}


	public boolean trajectoryFinished(){
		return mSwerveDriveHelper.hasFinishedTrajectory(getRobotPose());
	}


	public void stopTrajectory(){
		mPeriodicIO.state = SwerveState.DISABLED;
		mSwerveDriveHelper.stopTrajectory();
	}

///////////////////////////////////////  Misc Methods /////////////////////////////////////////////
	public boolean atTrajectoryReference(Pose2d targetPosition){//the trajectory reference is the only atReference method that uses the swerveDriveHelper because the trajectory is stored in SwerveDriveHelper
		return mSwerveDriveHelper.atTrajectoryReference(getRobotPose());
	}


	public boolean atReference(Pose2d targetPose, double xTolerance, double yTolerance, Rotation2d angleTolerance){
        return atTranslationReference(targetPose.getX(), targetPose.getY(), xTolerance, yTolerance) && atAngleReference(targetPose.getRotation(), angleTolerance);
	}


	public boolean atReference(Pose2d targetPose, double positionTolerance, Rotation2d angleTolerance){
        return atTranslationReference(targetPose.getX(), targetPose.getY(), positionTolerance) && atAngleReference(targetPose.getRotation(), angleTolerance);
	}


	public boolean atAngleReference(Rotation2d targetAngle, Rotation2d angleTolerance){
		return Math.abs(targetAngle.getRadians()-targetAngle.getRadians()) < angleTolerance.getRadians();
	}


	public boolean atTranslationReference(double targetX, double targetY, double xTolerance, double yTolerance){
		Pose2d robotPose = getRobotPose();
		return Math.abs(targetX-robotPose.getX()) < xTolerance && Math.abs(targetY-robotPose.getY()) < yTolerance;
	}


	public boolean atTranslationReference(double targetX, double targetY, double positionTolerance){
		return atTranslationReference(targetX, targetY, (Math.sqrt(2)/2)*positionTolerance, (Math.sqrt(2)/2)*positionTolerance);
	}
	

	public boolean atTranslationReference(Translation2d target, double positionTolerance){
		return atTranslationReference(target.getX(), target.getY(), positionTolerance);
	}
	

	public void setNeutralMode(NeutralMode driveMode, NeutralMode steerMode){
		for (SwerveModule m : mModules){
			m.setDriveNeutralMode(driveMode);
			m.setSteerNeutralMode(steerMode);
		} 
	}


	public void updateSteerPIDConstants(PIDConstants constants){
		for(SwerveModule m : mModules) m.updateSteerPIDConstants(constants);
	}


    public void updateDriveKS(double driveKS) {
		for(SwerveModule m : mModules) m.updateDriveKS(driveKS);
    }	


	public void updateDrivePIDConstants(PIDConstants constants){
		for(SwerveModule m : mModules) m.updateDrivePIDConstants(constants);
	}	


    public void updateSteerKS(double steerKS) {
		for(SwerveModule m : mModules) m.updateSteerKS(steerKS);
    }	


	public boolean allModulesAtTarget(){
		for(SwerveModule m : mModules) if(!m.angleOnTarget()) return false;
		return true;
	}


	public void setSnapAngle(Rotation2d target){
		mSwerveDriveHelper.setSnapAngle(getRobotRotation(), target);
	}

	
	public void setHoldAngle(Rotation2d target){
		mSwerveDriveHelper.setHoldAngle(getRobotRotation(), target);
	}


	@Override
	public void disable(){
		mPeriodicIO.state = SwerveState.DISABLED;
	}


	public void temporaryDisableRotation(){
		mSwerveDriveHelper.temporaryDisableRotation();
	}


	@Override
	public void stop() {
		mPeriodicIO.state = SwerveState.DISABLED;
		for(SwerveModule module : mModules) module.stop();// immediately stops the modules because if you call stop, you don't want to wait for writeOutputs
	}


	public SwerveRotationHelper.RotationHelperMode getRotationHelperMode(){
		return mSwerveDriveHelper.getRotationMode();
	}


	@Override
	public void writeOutputs() {
		SwerveModuleState[] moduleStates;

		switch(mPeriodicIO.state){
			case DISABLED:
				mSwerveDriveHelper.disableRotation();
				for(SwerveModule module : mModules) module.stop();
				break;
			case DRIVE_WITH_DODGE:
				moduleStates = mSwerveDriveHelper.getDriveAndDodge(mPeriodicIO.translationInput, mPeriodicIO.dodgeDirection, getRobotPose());
				runClosedLoop(moduleStates);
				break;
			case DRIVE:
				moduleStates = mSwerveDriveHelper.getDrive(mPeriodicIO.translationInput, mPeriodicIO.rotationInput, getRobotPose(), mPeriodicIO.robotCentric);
				runClosedLoop(moduleStates);
				break;
			case SNAP_TRANSLATION_AND_FACE_POINT:
				moduleStates = mSwerveDriveHelper.getSnapPositionAndFacePoint(mPeriodicIO.targetTranslation, mPeriodicIO.targetFacePoint, getRobotPose(), mPeriodicIO.facePointForwardDirection);
				runClosedLoop(moduleStates);
				break;
			case DRIVE_FACE_POINT:
				moduleStates = mSwerveDriveHelper.getDriveAndFacePoint(mPeriodicIO.translationInput, getRobotPose(), mPeriodicIO.targetFacePoint, mPeriodicIO.robotCentric, mPeriodicIO.facePointForwardDirection);
				runClosedLoop(moduleStates);
				break;
			case DRIVE_FACE_ANGLE:
				moduleStates = mSwerveDriveHelper.getDriveAndFaceAngle(mPeriodicIO.translationInput, getRobotPose(), mPeriodicIO.targetAngle, mPeriodicIO.robotCentric);
				runClosedLoop(moduleStates);
				break;
			case VISION:
				moduleStates = mSwerveDriveHelper.getVisionSnap(getRobotPose(), mPeriodicIO.targetPose);
				runClosedLoop(moduleStates);
				break;
			case POSITION:
				moduleStates = mSwerveDriveHelper.getSnapToPosition(getRobotPose(), mPeriodicIO.targetPose);
				runClosedLoop(moduleStates);
				break;
			case VISION_LOCK_X:
				moduleStates = mSwerveDriveHelper.getVisionLockX(mPeriodicIO.translationInput.getY(), mPeriodicIO.targetPose, getRobotPose());
				runClosedLoop(moduleStates);
				break;
			case FOLLOW_TRAJECTORY:
				moduleStates = mSwerveDriveHelper.getFollowTrajectory(getRobotPose());
				runClosedLoop(moduleStates);
				break;
			case LOCK_WHEELS:
				mModules[0].setAngle(Rotation2d.fromDegrees(45));
				mModules[1].setAngle(Rotation2d.fromDegrees(135));
				mModules[2].setAngle(Rotation2d.fromDegrees(225));
				mModules[3].setAngle(Rotation2d.fromDegrees(315));
				break;
			default:
				break;
		}
		mPeriodicIO.lastState = mPeriodicIO.state;
		mPeriodicIO.lastPose = getRobotPose();
		mPeriodicIO.lastTimeStamp = mPeriodicIO.timeStamp;
		mPeriodicIO.timeStamp = Timer.getFPGATimestamp();
		mPeriodicIO.dt = mPeriodicIO.timeStamp - mPeriodicIO.lastTimeStamp;
	}


	private void runOpenLoop(SwerveModuleState[] moduleStates){
		for(int i = 0; i < mModules.length; i++){
			mModules[i].setDesiredState(moduleStates[i], false);
		}
	}


	private void runClosedLoop(SwerveModuleState[] moduleStates){
		for(int i = 0; i < mModules.length; i++){
			mModules[i].setDesiredState(moduleStates[i], true);
		}
	}


	////////////////////  Getters  ///////////////////////////////////////////////////////////
	public Pose2d getTargetPose(){
		if(mPeriodicIO.state != SwerveState.POSITION) DriverStation.reportWarning("issue with getTargetPose", false);
		return mPeriodicIO.targetPose;
	}


	public SwerveModule[] getModules(){
		return mModules;
	}


	private Rotation2d getGyroYaw(){
		return Rotation2d.fromDegrees(mGyro.getYaw());
	}


	public double getAverageModuleSpeeds(){
		double sum = 0;
		for(SwerveModule m : mModules) sum += Math.abs(m.getVelocity());
		return sum / mModules.length;
	}


	public Rotation3d getRobotRotation3d(){
		return new Rotation3d(getRoll().getRadians(), getPitch().getRadians(), getRobotRotation().getRadians());
	}

	
	public SwerveState getLastState(){
		return mPeriodicIO.lastState;
	}


	public double getCurrentDraw(){
		double currentDraw = 0.0;
		for(SwerveModule module : mModules) currentDraw += module.getCurrentDraw();
		return currentDraw;
	}


	public double getVoltage(){
		double voltage = 0.0;
		for(SwerveModule module : mModules) voltage += module.getVoltage();
		return voltage / mModules.length;
	}


	public double getPowerConsumption(){
		double power = 0.0;
		for(SwerveModule module : mModules) power += module.getPowerConsumption();
		return power;
	}


	public Optional<Double> getTrajectoryPercentCompletion(){
		return mSwerveDriveHelper.getTrajectoryPercentCompletion();
	}

	
	public Rotation2d getRoll(){
		return Rotation2d.fromDegrees(mGyro.getPitch());
	}

	
	public Rotation2d getPitch(){
		return Rotation2d.fromDegrees(mGyro.getRoll());//if these values ever need an offset, do it with the config in phoenix tuner
	}


	public Pose2d getRobotPose(){
		return mPoseEstimator.getLatestPose();
	}


	public Rotation2d getRobotRotation(){
		return getRobotPose().getRotation();
	}


    public Rotation2d getRotationalSpeed() {
        return getRobotPose().getRotation().minus(mPeriodicIO.lastPose.getRotation()).div(mPeriodicIO.dt);
    }


    public double getTranslationalSpeed() {
		Pose2d robotPose = getRobotPose();
        return Math.hypot(robotPose.getX()-mPeriodicIO.lastPose.getX(), robotPose.getY()-mPeriodicIO.lastPose.getY()) / (mPeriodicIO.dt);
    }	

	
    public double getTrajectoryError() {
        return mSwerveDriveHelper.getTrajectoryError(getRobotPose().getTranslation());
    }


    public Pose2d getLimelightFieldCentricPose() {
        Pose2d robotPose = getRobotPose();
        return new Pose2d(robotPose.getTranslation(), robotPose.getRotation().rotateBy(Rotation2d.fromDegrees(180))).transformBy(new Transform2d(new Translation2d(VisionConstants.kFrontLimelightForward, -VisionConstants.kFrontLimelightRight), Rotation2d.fromDegrees(0)));
    }


///////////////////  Telemetry and Logging  //////////////////////////////////////////////////////
	@Override
	public void outputTelemetry() {
		// System.out.println(" Robot pose: " + getRobotPose());
	}
}