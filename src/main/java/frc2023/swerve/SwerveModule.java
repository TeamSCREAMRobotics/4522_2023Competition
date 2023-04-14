package frc2023.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.team4522.lib.deviceConfiguration.DeviceUtil;
import com.team4522.lib.logging.CSVWriteable;
import com.team4522.lib.pid.PIDConstants;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc2023.Constants.*;

public class SwerveModule implements CSVWriteable {
	
	private final TalonFX mSteer, mDrive;
	private final CANCoder mEncoder;
	private final SwerveModuleConstants mModuleConstants;
	/**mutliply this by native drive position to get drive position in meters */
	public final double kDrivePositionCoefficient;
	/**mutliply this by native drive Velocity to get drive Velocity in meters per second */
	public final double kDriveVelocityCoefficient;
	/**mutliply this by native steer native position to get degrees */
	public final double kSteerPositionCoefficient;
	/**mutliply this by steer native velocity to get degrees per second */
	public final double kSteerVelocityCoefficient;

	/** instead of messing around with limiting the acceleration of the robot with profiledPIDControllers or slew rate limiters, we just limit the acceleration of the drive
		motor here. There is no situation where we wouldn't want to accceleration limit the drive motor, so it makes sense to do it in the swerve module */
	private final SlewRateLimiter mDriveAccelerationLimiter = new SlewRateLimiter(SwerveModuleConstants.kDriveSlewRate, -SwerveModuleConstants.kDriveSlewRate, 0);
	
	private SwerveModuleState mDesiredState = new SwerveModuleState();

	public SwerveModule(TalonFX drive, TalonFX steer, CANCoder cancoder, SwerveModuleConstants moduleConstants){
		mDrive = drive;
		mSteer = steer;
		mEncoder = cancoder;
		mModuleConstants = moduleConstants;
		kDrivePositionCoefficient = 1.0 / mModuleConstants.kDriveTicksPerRotation / mModuleConstants.kDriveGearRatio * mModuleConstants.kWheelCircumference;
		kSteerPositionCoefficient = 1.0 / mModuleConstants.kSteerTicksPerRotation / mModuleConstants.kSteerGearRatio * 360.0;

		kDriveVelocityCoefficient = kDrivePositionCoefficient * 10.0;
		kSteerVelocityCoefficient = kSteerPositionCoefficient * 10.0;

		DeviceUtil.configTalonFXPID(mSteer, mModuleConstants.steerPIDConstants, true);
		DeviceUtil.configTalonFXPID(mDrive, mModuleConstants.drivePIDConstants, true);
		DeviceUtil.configTalonFXMotionMagic(mSteer, mModuleConstants.steerMotionMagicConstants, true);

		resetSteerToAbsolutePosition();
		mDrive.setSelectedSensorPosition(0.0);
	}

	/**
	 * 
	 * 
	 * 	 * This method converts the desired state from the kinematics(which has a bounded rotation) and converts it to the nearest unbounded angle(so that we can use 
	 * motionmagic for the steer motors). It also reverses the modules if needed for the shortest module rotation 
	 * 
	 * @param desiredState The desired state of the module (generated from WPI's SwerveDriveKinematics)
	 * @param currentAngleUnbounded The unbounded angle of the motor. It needs to be unbounded because we are using the onboard sensor on the TalonFX and we have to tell 
	 *  	the motor to go to its nearest multiple of 360(180 with reversing module) angle. For example, if the module is reading 720 degrees and our desired state is 45 degrees,
	 * 		the method would return (720 + 45) degrees
	 * @return The SwerveModuleState with the shortest, unbounded angle to get to the module's desired state
	 * 
	 */
	private static SwerveModuleState desiredStateToNativeState(SwerveModuleState desiredState, Rotation2d currentAngleUnbounded){
		double deltaDegrees = desiredState.angle.minus(currentAngleUnbounded).getDegrees();
		boolean invert = Math.abs(deltaDegrees) > 90;
		if(invert) deltaDegrees -= Math.signum(deltaDegrees)*180;

		return new SwerveModuleState(
			(invert ? -1 : 1) * desiredState.speedMetersPerSecond,
			Rotation2d.fromDegrees(currentAngleUnbounded.getDegrees() + deltaDegrees)
		);
	}

	/**
	 * This method sets the modules to move to their desired state
	 * @param closedLoop if we are in closed loop control, we run the motors in velocity mode, otherwise we run them in percentoutput
	 * @param deadbandWithMinSpeed If the speed of the swerve is below a minimum value, we don't run the drive or steer motors. If this value is false, the motors will run regardless of how low the speed is.
	 * We use this for locking the wheels, where we want the modules to turn but we want the speed to be 0.
	 */
	public void setDesiredState(SwerveModuleState moduleState, boolean closedLoop, boolean deadbandWithMinSpeed){
		mDesiredState = desiredStateToNativeState(moduleState, Rotation2d.fromDegrees(getUnboundedAngle()));

		if(Math.abs(mDesiredState.speedMetersPerSecond) < SwerveConstants.kMinSpeed && deadbandWithMinSpeed){
			mSteer.neutralOutput();
			mDrive.neutralOutput();
		} else{
			setSteerMotor(mDesiredState.angle);
			setDriveMotor(closedLoop, mDesiredState.speedMetersPerSecond);
		}
	}
	
	public void setDesiredState(SwerveModuleState moduleState, boolean closedLoop){
		setDesiredState(moduleState, closedLoop, true);
	}

	private void setSteerMotor(Rotation2d angle){
		double steerTarget = angle.getDegrees()/kSteerPositionCoefficient;
		if(!angleOnTarget()) mSteer.set(ControlMode.MotionMagic, steerTarget, DemandType.ArbitraryFeedForward,  Math.signum(steerTarget-mSteer.getSelectedSensorPosition())*mModuleConstants.steerKS);
		else mSteer.set(ControlMode.PercentOutput, 0);
	}

	private void setDriveMotor(boolean closedLoop, double speedMetersPerSecond){
		if(closedLoop){
			mDrive.set(ControlMode.Velocity, mDriveAccelerationLimiter.calculate(speedMetersPerSecond)/kDriveVelocityCoefficient, DemandType.ArbitraryFeedForward, Math.signum(speedMetersPerSecond)*mModuleConstants.driveKS);
		}
		else{

			mDrive.set(ControlMode.PercentOutput, speedMetersPerSecond/SwerveConstants.kMaxSpeed, DemandType.ArbitraryFeedForward, Math.signum(speedMetersPerSecond)*mModuleConstants.driveKS);
		}
	}

	/**
	 * Used to set the angle, but not move the drive motors. We use this for locking the wheels when the swerve isn't moving
	 */
	public void setAngle(Rotation2d angle){
		setDesiredState(new SwerveModuleState(0, angle), true, false);
	}

	public void stop(){
		mSteer.neutralOutput();
		mDrive.neutralOutput();
	}

	public void setDriveNeutralMode(NeutralMode mode){
		mDrive.setNeutralMode(mode);
	}

	public void setSteerNeutralMode(NeutralMode mode){
		mSteer.setNeutralMode(mode);
	}

	public String name(){
		return mModuleConstants.name;
	}

	public void updateDrivePIDConstants(PIDConstants constants){
		mModuleConstants.drivePIDConstants = constants;
		DeviceUtil.configTalonFXPID(mDrive, mModuleConstants.drivePIDConstants, false);
	}
	
    public void updateDriveKS(double driveKS) {
		mModuleConstants.driveKS = driveKS;
    }

	public void updateSteerPIDConstants(PIDConstants constants){
		mModuleConstants.steerPIDConstants = constants;
		DeviceUtil.configTalonFXPID(mSteer, mModuleConstants.steerPIDConstants, false);
	}

    public void updateSteerKS(double steerKS) {
		mModuleConstants.steerKS = steerKS;
    }

	public double getCurrentDraw(){
		return mDrive.getSupplyCurrent() + mSteer.getSupplyCurrent();
	}

	public double getVoltage(){
		return (mDrive.getBusVoltage() + mSteer.getBusVoltage()) / 2;
	}

	public double getPowerConsumption(){
		return mDrive.getSupplyCurrent()*mDrive.getBusVoltage() + mSteer.getSupplyCurrent()*mSteer.getBusVoltage();
	}

	public boolean angleOnTarget(){
		return Math.abs(mDesiredState.angle.minus(getAbsoluteAngle()).getDegrees()) < mModuleConstants.kAngleOnTargetThreshold.getDegrees();
	}

	/**
	 * Takes the CANCoder reading with the offset and resets the position of the steer motor. This is the only place we use the CANCoder because the onboard motion magic is faster
	 */
	public void resetSteerToAbsolutePosition(){
		mSteer.setSelectedSensorPosition(getAngleFromCANCoder().getDegrees()/kSteerPositionCoefficient);
	} 

	public Rotation2d getAngleFromCANCoder(){
		return Rotation2d.fromDegrees(mEncoder.getAbsolutePosition()).minus(mModuleConstants.offset);
	}

	public Rotation2d getAbsoluteAngle(){
		return ScreamUtil.boundRotation(Rotation2d.fromDegrees(getUnboundedAngle()));
	}

	private double getUnboundedAngle(){
		return mSteer.getSelectedSensorPosition()*kSteerPositionCoefficient;
	}

	public double getSteerError(){
		return mSteer.getClosedLoopError()*kSteerPositionCoefficient;
	}
	
	public double getDriveError(){
		return mDrive.getClosedLoopError()*kDriveVelocityCoefficient;
	}

	public double getDistance(){//distance for the drive motor in meters
		return mDrive.getSelectedSensorPosition()*kDrivePositionCoefficient;
	}

	public double getVelocity(){
		return mDrive.getSelectedSensorVelocity()*kDriveVelocityCoefficient;
	}

	public SwerveModuleState getState(){
		return new SwerveModuleState(getVelocity(), getAbsoluteAngle());
	}

	public SwerveModuleState getDesiredState(){
		return mDesiredState;
	}

	public SwerveModulePosition getPosition(){
		return new SwerveModulePosition(getDistance(), getAbsoluteAngle());
	}

	@Override
	public String toCSV() {
		return String.join(", ", mDesiredState.toString(), getState().toString());
	}

	public Rotation2d getRawEncoder() {
		return Rotation2d.fromDegrees(mEncoder.getAbsolutePosition());
	}
}