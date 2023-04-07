package frc2023.swerve;

import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import frc2023.Constants;
import frc2023.Constants.*;

/**A helper class designed to make rotation with the swerve easier */
public class SwerveRotationHelper {
	
	private RotationHelperMode mode = RotationHelperMode.DISABLED;
	private final Timer openLoopTimer = new Timer();
	private final PIDController mSnapController;//We intentionally use PID controllers and not ProfiledPIDControllers because they are faster and snappier
	private final PIDController mHoldController;

	public enum RotationHelperMode {
		DISABLED, SNAP, HOLD, OPEN_LOOP
	}

	private Rotation2d mTargetAngle = SwerveConstants.robotForwardAngle;

	public SwerveRotationHelper(){
		mSnapController = ScreamUtil.createPIDController(SwerveConstants.snapRotationPIDConstants, Constants.kSubsystemPeriodSeconds);
		mSnapController.enableContinuousInput(-Math.PI, Math.PI);

		mHoldController = ScreamUtil.createPIDController(SwerveConstants.holdRotationPIDConstants, Constants.kSubsystemPeriodSeconds);
		mHoldController.enableContinuousInput(-Math.PI, Math.PI);
	}

	/**
	 * Makes the swerve aggressively rotate to get to a target angle
	 */
	public void setSnap(Rotation2d measured, Rotation2d target){
		mTargetAngle = target;
		mode = RotationHelperMode.SNAP;
	}
	
	/**
	 * Sets the robot to try to hold a given angle with PID. The goal is not to move, or rotate the robot, but to keep it in a position while resisting outside forces
	 */
	public void setHold(Rotation2d measured, Rotation2d target){
		mTargetAngle = target;
		mode = RotationHelperMode.HOLD;
	}

	/**
	 * Open loop control for the rotation of the swerve. This is used when the driver is using the turn joystick
	 */
	public void setOpenLoop(){
		openLoopTimer.reset();
		openLoopTimer.start();
		mode = RotationHelperMode.OPEN_LOOP;
	}
	
	public void disable(){
		mode = RotationHelperMode.DISABLED;
	}

	/**
	 * @param measured the measured robot angle
	 * @param manualRotationInput the manual rotation input is only for open loop control. If using snap or hold, make it 0.0
	 */
	public double calculateRotation(Rotation2d measured, double manualRotationInput){
		switch(mode){
			case OPEN_LOOP://timer to wait for the robot to stop after an open loop turn is finished. Once the robot has stopped, we set it in hold mode so that it will stay in the same direction if being pushed or if driving without turn input
				if(openLoopTimer.get() > SwerveConstants.kRotationOpenLoopDuration) {
					openLoopTimer.stop();
					setHold(measured, measured);
				}
				return manualRotationInput;
			case HOLD:
				return mHoldController.calculate(measured.getRadians(), mTargetAngle.getRadians());
			case SNAP:
				return mSnapController.calculate(measured.getRadians(), mTargetAngle.getRadians());
			default:
			case DISABLED:
				return 0;
		}
	}

	public RotationHelperMode getMode(){
		return mode;
	}

	public Rotation2d getTargetAngle(){
		return mTargetAngle;
	}
}