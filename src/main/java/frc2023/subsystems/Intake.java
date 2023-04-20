package frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.team4522.lib.deviceConfiguration.DeviceUtil;
import com.team4522.lib.util.TimeBoundIncrementor;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc2023.Constants;
import frc2023.Constants.*;
import frc2023.Constants.IntakeConstants.ConveyorConstants;
import frc2023.Constants.IntakeConstants.LowerConveyorConstants;
import frc2023.Constants.IntakeConstants.RodConstants;
import frc2023.Constants.IntakeConstants.ShooterConstants;
import frc2023.Constants.IntakeConstants.UpperConveyorConstants;
import frc2023.PlacementStates.Level;
import frc2023.shuffleboard.tabs.TestTab;

/** This our intake subsystem. It has a four bar intake(solenoid and roller) in the front for intaking cubes, a hopper in the middle to index the cubes(upper and lower conveyor),
 * and a couple of flywheels on the back(the poop shooter). 
 * 
 * <p> We primarily shoot cubes out the front of our intake by reversing the upper and lower conveyor motors and running the
 * roller in like normal. This pops the cube out the top of the intake, shooting the cubes.
 * 
 * <p> All of these different devices work together, are only for cubes, and we can only hold 1 game piece at a time. We put
 * all of these devices one subsystem because there is no case where we want different parts of this subsystem to do independent things
 */
public class Intake extends Subsystem{

	public final PeriodicIO mPeriodicIO = new PeriodicIO();
	private final CANSparkMax mRollerMotor;
	private final Solenoid mSolenoid;
    private final Devices mDevices = Devices.getInstance();
	private final CANSparkMax mUpperConveyorMotor;
	private final CANSparkMax mLowerConveyorMotor;
	private final TalonSRX mShooterMotor;
	private final TalonSRX mRodMotor;
	private final DigitalInput mBeamBreak;
	
	private Intake(){
		mSolenoid = mDevices.dIntakeSolenoid;
		mRollerMotor = mDevices.dIntakeMotor;
		mUpperConveyorMotor = mDevices.dUpperConveyorMotor;
		mLowerConveyorMotor = mDevices.dLowerConveyorMotor;
        mShooterMotor = mDevices.dShooterMotor;
		mRodMotor = mDevices.dRodMotor;
		mBeamBreak = mDevices.dBeamBreak;

        DeviceUtil.configTalonSRXPID(mRodMotor, RodConstants.kRodOutPID, true, 0);
        DeviceUtil.configTalonSRXPID(mRodMotor, RodConstants.kRodInPID, true, 1);

        DeviceUtil.configTalonSRXMotionMagic(mRodMotor, RodConstants.outMotionMagicConstants, true);
		mRodMotor.configForwardSoftLimitEnable(true);
		mRodMotor.configReverseSoftLimitEnable(true);
	}

	
	private static Intake mInstance = null;
	public static Intake getInstance(){
		if(mInstance == null){
			mInstance = new Intake();
		}
		return mInstance;
	}


	public static enum IntakeState{//we have a lot of control states for the intake.
		DISABLED, RETRACT, EXTEND, INTAKE, EJECT, RETRACT_AND_RUN, SHOOT_CUBE_HIGH, SHOOT_CUBE_MID, BACKWARDS_EJECT, SWEEP, INTAKE_FOR_POOPSHOOT, 
						EJECT_ONLY_LOWER_CONVEYOR, POOP_SHOOT_FROM_CHARGE_LINE, PREPARE_FOR_SHOT,
			SHOOT_CUBE_HIGH_AUTO, SHOOT_CUBE_MID_AUTO, INAKE_AUTO, POOP_SHOOT_FROM_CHARGE_LINE_AUTO, POOP_SHOOT_FROM_CHARGE_LINE_AUTO_WITHOUT_ROD, EXTEND_ROD;
		
		public boolean isPoopShootState(){
			return this == BACKWARDS_EJECT || this == POOP_SHOOT_FROM_CHARGE_LINE || this == POOP_SHOOT_FROM_CHARGE_LINE_AUTO || this == POOP_SHOOT_FROM_CHARGE_LINE_AUTO_WITHOUT_ROD;
		}
	}


	public class PeriodicIO{
		//Inputs
		public IntakeState lastState = IntakeState.RETRACT;
		public TimeBoundIncrementor timeBeforeWheelsCanRun = new TimeBoundIncrementor(0.0, IntakeConstants.kWaitBeforeRunWheelsDuration, IntakeConstants.kWaitBeforeRunWheelsDuration);
		public TimeBoundIncrementor timeBeforeCanShoot = new TimeBoundIncrementor(0.0, ShooterConstants.kTimeBeforeCanShoot, ShooterConstants.kTimeBeforeCanShoot);

		//Outputs
		public IntakeState state = IntakeState.RETRACT;
		public boolean isExtended = false;
		public double rollerPercentOutput = 0.0;
		public double upperConveyorPercentOutput = 0.0;
		public double lowerConveyorPercentOutput = 0.0;
		public double shooterPercentOutput = 0.0;
		public boolean rodExtended = false;
		public boolean rodExtendedLastLoop = false;
	}

//////////////////////////// Methods that set the intake state ////////////////////////////////////////////////////////////////////////

	private final Timer retractTimer = new Timer();
	public void retract(){// this is the retract logic for teleop; If we retract immediately, the intake will sometimes get jammed on the cube and get stuck outside of frame perimeter.
							//When we are not telling the intake to do anything in teleop, we call retract, it runs the wheels for a little longer, then actually reaches the retract state.
		if(mPeriodicIO.lastState != IntakeState.INTAKE && mPeriodicIO.lastState != IntakeState.RETRACT_AND_RUN){
			mPeriodicIO.state = IntakeState.RETRACT;
			return;
		} else if(mPeriodicIO.lastState != IntakeState.RETRACT_AND_RUN){
			retractTimer.reset();
			retractTimer.start();
			mPeriodicIO.state = IntakeState.RETRACT_AND_RUN;
		}
		if(retractTimer.get() >= IntakeConstants.kRunWhileRetractedDuration){
			retractTimer.stop();
			mPeriodicIO.state = IntakeState.RETRACT;
		} else{
			mPeriodicIO.state = IntakeState.RETRACT_AND_RUN;
		}
	}


	public void intake(){
		mPeriodicIO.state = IntakeState.INTAKE;
	}


	public void eject(){
		mPeriodicIO.state = IntakeState.EJECT;
	}
	

	public void shootCubeHigh(){
		mPeriodicIO.state = IntakeState.SHOOT_CUBE_HIGH;
	}

	
	public void shootCubeMid(){
		mPeriodicIO.state = IntakeState.SHOOT_CUBE_MID;
	}


	private void shootCubeHighAuto() {
		mPeriodicIO.state = IntakeState.SHOOT_CUBE_HIGH_AUTO;
	}


	public void shootCubeMidAuto() {
		mPeriodicIO.state = IntakeState.SHOOT_CUBE_MID_AUTO;
	}

	public void shootCube(Level level){
		switch(level){
			case HYBRID:
				eject();
				break;
			case MIDDLE:
				shootCubeMid();
				break;
			case TOP:
				shootCubeHigh();
				break;
		}
	}


	@Override
	public void disable(){
		mPeriodicIO.state = IntakeState.DISABLED;
	}


	public void extend(){
		mPeriodicIO.state = IntakeState.EXTEND;
	}


	public void poopShootFromChargeLine(){
		mPeriodicIO.state = IntakeState.POOP_SHOOT_FROM_CHARGE_LINE;
	}


	public void intakeForPoopshoot(){
		mPeriodicIO.state = IntakeState.INTAKE_FOR_POOPSHOOT;
	}


	public void sweep(){
		mPeriodicIO.state = IntakeState.SWEEP;
	}


	public void ejectOnlyLowerConveyor(){
		mPeriodicIO.state = IntakeState.EJECT_ONLY_LOWER_CONVEYOR;
	}


	public void backwardsEject(){
		mPeriodicIO.state = IntakeState.BACKWARDS_EJECT;
	}


	public void intakeDuringAuto(){
		mPeriodicIO.state = IntakeState.INAKE_AUTO;
	}


	public void poopShootFromChargeLineAuto(){
		mPeriodicIO.state = IntakeState.POOP_SHOOT_FROM_CHARGE_LINE_AUTO;
	}

	
	public void poopShootFromChargeLineAutoWithoutRod(){
		mPeriodicIO.state = IntakeState.POOP_SHOOT_FROM_CHARGE_LINE_AUTO_WITHOUT_ROD;
	}

	public void extendRod(){
		mPeriodicIO.state = IntakeState.EXTEND_ROD;
	}


	@Override
	public void stop() {
		mPeriodicIO.state = IntakeState.DISABLED;
		mRollerMotor.stopMotor();
		mUpperConveyorMotor.stopMotor();
		mLowerConveyorMotor.stopMotor();
	}


	public void prepareForShot() {
		mPeriodicIO.state = IntakeState.PREPARE_FOR_SHOT;
	}


	public void setDesiredState(IntakeState desiredState){
		switch(desiredState){
			case DISABLED:
				disable();
				break;
			case RETRACT:
				retract();
				break;
			case EXTEND:
				extend();
				break;
			case INTAKE:
				intake();
				break;
			case EJECT:
				eject();
				break;
			case SHOOT_CUBE_HIGH:
				shootCubeHigh();
				break;
			case SHOOT_CUBE_MID:
				shootCubeMid();
				break;
			case BACKWARDS_EJECT:
				backwardsEject();
				break;
			case EJECT_ONLY_LOWER_CONVEYOR:
				ejectOnlyLowerConveyor();
				break;
			case POOP_SHOOT_FROM_CHARGE_LINE:
				poopShootFromChargeLine();
				break;
			case INTAKE_FOR_POOPSHOOT:
				intakeForPoopshoot();
				break;
			case PREPARE_FOR_SHOT:
				prepareForShot();
				break;
			case SHOOT_CUBE_HIGH_AUTO:
				shootCubeHighAuto();
				break;
			case SHOOT_CUBE_MID_AUTO:
				shootCubeMidAuto();
				break;
			case INAKE_AUTO:
				intakeDuringAuto();
				break;
			case POOP_SHOOT_FROM_CHARGE_LINE_AUTO:
				poopShootFromChargeLineAuto();
				break;
			case POOP_SHOOT_FROM_CHARGE_LINE_AUTO_WITHOUT_ROD:
				poopShootFromChargeLineAutoWithoutRod();
			case EXTEND_ROD:
				extendRod();
			default:
			DriverStation.reportError("Wrong/Unimplemented IntakeState Chosen in setDesiredState()", false);
		}
	}

	////////////////////////////////////////////////// Writing Outputs ////////////////////////////////////////////////////////////////////////////////////////////////
	private double lastTimeStamp;
	
	private double intakeModeBeamBrokenTimeStamp = -Integer.MAX_VALUE;
	boolean beamBrokenTimestampRegistered = false;

	@Override
	public void writeOutputs() {
		double timeStamp = Timer.getFPGATimestamp();
		double dt = timeStamp - lastTimeStamp;
		boolean wheelsCanRun = mPeriodicIO.timeBeforeWheelsCanRun.atMinimum();//the wheels can't run while the intake is fully in, so the wheelsCanRun variable is true only when the intake is out of the robot enough
		boolean canShoot = mPeriodicIO.timeBeforeCanShoot.atMinimum(); // canShoot is true when the flywheel has had enough time to rev up for the shot.

		if(mPeriodicIO.state != IntakeState.INTAKE){//when the driver is intaking, the wheels stop when the beam break is tripped, but continue again if the driver keeps holding intake for long enough. 
			beamBrokenTimestampRegistered = false;	//the beam broken timestamp registers when the beam is broken, and in the intake state, we check the current time in relation to when the beambroken was tripped for whether to run the wheels
		}

		switch(mPeriodicIO.state){
			case DISABLED:
			case RETRACT:
				setPeriodicOutputsToZero();
				mPeriodicIO.isExtended = false;
				mPeriodicIO.rodExtended = false;
				break;
			case EXTEND:
				setPeriodicOutputsToZero();
				mPeriodicIO.isExtended = true;
				mPeriodicIO.rodExtended = false;
				break;
			case INTAKE_FOR_POOPSHOOT://when we intake for poopshoot, we run everything at max because the cube needs to go all the ways in the robot. We also ignore the beam break.
				mPeriodicIO.isExtended = true;
				mPeriodicIO.rodExtended = false;
				mPeriodicIO.shooterPercentOutput = 0.0;
				mPeriodicIO.rollerPercentOutput = 1.0;
				mPeriodicIO.upperConveyorPercentOutput = 1.0;
				mPeriodicIO.lowerConveyorPercentOutput = 1.0;
				break;
			case INTAKE:
				mPeriodicIO.isExtended = true;
				mPeriodicIO.rodExtended = false;
				if(wheelsCanRun){
					if(beamBroken() && !beamBrokenTimestampRegistered){
						intakeModeBeamBrokenTimeStamp = Timer.getFPGATimestamp();
						beamBrokenTimestampRegistered = true;
					} 

					double beamBrokenDt = Timer.getFPGATimestamp() - intakeModeBeamBrokenTimeStamp;

					if(beamBrokenTimestampRegistered && beamBrokenDt > ConveyorConstants.timeAfterBeamBreakToStop && beamBrokenDt < ConveyorConstants.timeAfterBeamBreakToStartAgain){
						mPeriodicIO.upperConveyorPercentOutput = 0.0;
						mPeriodicIO.lowerConveyorPercentOutput = 0.0;
					} else{
						mPeriodicIO.upperConveyorPercentOutput = UpperConveyorConstants.kIntakePO;
						mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kIntakePO;
					}
					
					mPeriodicIO.rollerPercentOutput = IntakeConstants.kIntakePO;
					mPeriodicIO.shooterPercentOutput = 0.0;
				} else{
					setPeriodicOutputsToZero();
				} 
				break;
			case EJECT:
				mPeriodicIO.isExtended = true;
				mPeriodicIO.rodExtended = false;
				if(wheelsCanRun){
					mPeriodicIO.rollerPercentOutput = IntakeConstants.kEjectPO;
					mPeriodicIO.upperConveyorPercentOutput = UpperConveyorConstants.kEjectPO;
					mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kEjectPO;
					mPeriodicIO.shooterPercentOutput = 0.0;
				} else{
					setPeriodicOutputsToZero();
				} 
				break;
			case RETRACT_AND_RUN:
				mPeriodicIO.isExtended = false;
				mPeriodicIO.rodExtended = false;

				mPeriodicIO.rollerPercentOutput = IntakeConstants.kRetractAndRunPO;
				mPeriodicIO.upperConveyorPercentOutput = UpperConveyorConstants.kRetractAndRunPO;
				mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kRetractAndRunPO;
				mPeriodicIO.shooterPercentOutput = 0.0;
				break;
			case SHOOT_CUBE_HIGH:
				mPeriodicIO.isExtended = true;
				mPeriodicIO.rodExtended = false;

				if(wheelsCanRun){
					if(Constants.includeDebugTabs){
						mPeriodicIO.rollerPercentOutput = TestTab.getInstance().shootingSpeedRoller.getDouble(0);//we use these instead if we want to tune values from shuffleboard. In future years, we should have a better tuning setup.
						mPeriodicIO.upperConveyorPercentOutput = TestTab.getInstance().shootingSpeedTop.getDouble(0);
						mPeriodicIO.lowerConveyorPercentOutput = TestTab.getInstance().shootingSpeedBottom.getDouble(0);
						mPeriodicIO.shooterPercentOutput = TestTab.getInstance().shootingSpeedPoopShooter.getDouble(0.0);
					} else{
						mPeriodicIO.rollerPercentOutput = IntakeConstants.kShootHighPO;
						mPeriodicIO.upperConveyorPercentOutput = UpperConveyorConstants.kShootHighPO;
						mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kShootHighPO;
						mPeriodicIO.shooterPercentOutput = 0.0;
					}
					
				} else{
					setPeriodicOutputsToZero();
				} 
				break;
			case SHOOT_CUBE_MID:
				mPeriodicIO.isExtended = true;
				mPeriodicIO.rodExtended = false;

				if(wheelsCanRun){
					mPeriodicIO.rollerPercentOutput = IntakeConstants.kShootMidPO;
					mPeriodicIO.upperConveyorPercentOutput = UpperConveyorConstants.kShootMidPO;
					mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kShootMidPO;
					mPeriodicIO.shooterPercentOutput = 0.0;
				} else{
					setPeriodicOutputsToZero();
				}
				break;
			case SHOOT_CUBE_HIGH_AUTO:
				mPeriodicIO.isExtended = true;
				mPeriodicIO.rodExtended = false;

				if(wheelsCanRun){
					mPeriodicIO.rollerPercentOutput = IntakeConstants.kShootHighAutoPO;
					mPeriodicIO.upperConveyorPercentOutput = UpperConveyorConstants.kShootHighAutoPO;
					mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kShootHighAutoPO;
					mPeriodicIO.shooterPercentOutput = 0.0;
				} else{
					setPeriodicOutputsToZero();
				}
				break;
			case SHOOT_CUBE_MID_AUTO:
				mPeriodicIO.isExtended = true;
				mPeriodicIO.rodExtended = false;
				if(wheelsCanRun){
					mPeriodicIO.rollerPercentOutput = IntakeConstants.kShootMidAutoPO;
					mPeriodicIO.upperConveyorPercentOutput = UpperConveyorConstants.kShootMidAutoPO;
					mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kShootMidAutoPO;
					mPeriodicIO.shooterPercentOutput = 0.0;
				} 
				else{
					setPeriodicOutputsToZero();
				}
				break;
			case BACKWARDS_EJECT:
				mPeriodicIO.isExtended = false;
				mPeriodicIO.rodExtended = false;
				mPeriodicIO.rollerPercentOutput = 0.0;
				mPeriodicIO.upperConveyorPercentOutput = UpperConveyorConstants.kBackwardsEjectPO;
				mPeriodicIO.shooterPercentOutput = ShooterConstants.kBackwardsEjectPO;
				if(canShoot){
					mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kBackwardsEjectPO;				
				} else{
					mPeriodicIO.lowerConveyorPercentOutput = -0.3;
				}
				break;
			case SWEEP:
				mPeriodicIO.isExtended = true;
				mPeriodicIO.rodExtended = false;
				mPeriodicIO.rollerPercentOutput = IntakeConstants.kSweepPO;
				mPeriodicIO.lowerConveyorPercentOutput = 0.0;
				mPeriodicIO.upperConveyorPercentOutput = 0.0;
				mPeriodicIO.shooterPercentOutput = 0.0;
				break;
			case POOP_SHOOT_FROM_CHARGE_LINE:
				mPeriodicIO.isExtended = false;
				mPeriodicIO.rodExtended = true;
				mPeriodicIO.rollerPercentOutput = 0.0;
				mPeriodicIO.upperConveyorPercentOutput = 0.0;
				if(canShoot){
					mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kPoopShootFromChargeLinePO;
				} else{
					mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kPreparePoopShoot;
				}
				mPeriodicIO.shooterPercentOutput = ShooterConstants.kPoopShootFromChargeLinePO;
				break;
			case POOP_SHOOT_FROM_CHARGE_LINE_AUTO:
				mPeriodicIO.isExtended = false;
				mPeriodicIO.rodExtended = true;
				mPeriodicIO.rollerPercentOutput = 0.0;
				mPeriodicIO.upperConveyorPercentOutput = 0.0;
				if(canShoot){
					mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kPoopShootFromChargeLineAutoPO;
				} else{
					mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kPreparePoopShoot;
				}
				mPeriodicIO.shooterPercentOutput = ShooterConstants.kPoopShootFromChargeLineAutoPO;
				break;
			case POOP_SHOOT_FROM_CHARGE_LINE_AUTO_WITHOUT_ROD:
				mPeriodicIO.isExtended = false;
				mPeriodicIO.rodExtended = false;
				mPeriodicIO.rollerPercentOutput = 0.0;
				mPeriodicIO.upperConveyorPercentOutput = 0.0;
				if(canShoot){
					mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kPoopShootFromChargeLineAutoPO;
				} else{
					mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kPreparePoopShoot;
				}
				mPeriodicIO.shooterPercentOutput = ShooterConstants.kPoopShootFromChargeLineAutoWithoutRodPO;
				break;
			case EJECT_ONLY_LOWER_CONVEYOR:
				mPeriodicIO.isExtended = false;
				mPeriodicIO.rodExtended = false;
				mPeriodicIO.rollerPercentOutput = 0.0;
				mPeriodicIO. upperConveyorPercentOutput = 0.0;
				mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kEjectOnlyLowerConveyorPO;
				mPeriodicIO.shooterPercentOutput = 0.0;
				break;
			case INAKE_AUTO:
				mPeriodicIO.isExtended = true;
				mPeriodicIO.rodExtended = false;
				if(wheelsCanRun){
					mPeriodicIO.isExtended = true;
					mPeriodicIO.rollerPercentOutput = IntakeConstants.kAutoIntakePO;
					mPeriodicIO.upperConveyorPercentOutput = UpperConveyorConstants.kAutoIntakePO;
					mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kAutoIntakePO;
					mPeriodicIO.shooterPercentOutput = 0.0;
				} else{
					setPeriodicOutputsToZero();
				} 
				break;
			case PREPARE_FOR_SHOT:
				mPeriodicIO.isExtended = true;
				mPeriodicIO.rodExtended = false;
				mPeriodicIO.rollerPercentOutput = 0.0;
				mPeriodicIO.upperConveyorPercentOutput = 0.0;
				mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kPrepareForShot;
				mPeriodicIO.shooterPercentOutput = 0.0;
				break;
			case EXTEND_ROD:
				mPeriodicIO.isExtended = false;
				mPeriodicIO.rodExtended = true;
				mPeriodicIO.rollerPercentOutput = 0.0;
				mPeriodicIO.upperConveyorPercentOutput = 0.0;
				mPeriodicIO.lowerConveyorPercentOutput = 0.0;
				mPeriodicIO.shooterPercentOutput = 0.0;
		}

		mSolenoid.set(mPeriodicIO.isExtended);
		mRollerMotor.set(mPeriodicIO.rollerPercentOutput);

		mUpperConveyorMotor.set(mPeriodicIO.upperConveyorPercentOutput);
		mLowerConveyorMotor.set(mPeriodicIO.lowerConveyorPercentOutput);
		mShooterMotor.set(ControlMode.PercentOutput, mPeriodicIO.shooterPercentOutput);
		setRodExtended(mPeriodicIO.rodExtended);

		if(mPeriodicIO.isExtended){
			mPeriodicIO.timeBeforeWheelsCanRun.increment(-dt);
		} else{
			mPeriodicIO.timeBeforeWheelsCanRun.increment(dt);
		}

		if(mPeriodicIO.state.isPoopShootState()){
			mPeriodicIO.timeBeforeCanShoot.increment(-dt);
		}  else{
			mPeriodicIO.timeBeforeCanShoot.increment(dt);
		}

		mPeriodicIO.lastState = mPeriodicIO.state;
		mPeriodicIO.rodExtendedLastLoop = mPeriodicIO.rodExtended;
		lastTimeStamp = timeStamp;
	}

	private void setRodExtended(boolean extended){
		if(extended != mPeriodicIO.rodExtendedLastLoop){//toggle, if rod extension changes, reconfigure the motionmagic. It would be better if the rod had an auxilary MotionMagic slot in addition to its auxilary PID
			if(extended){
				mRodMotor.selectProfileSlot(0, 0);
				mRodMotor.configMotionAcceleration(RodConstants.outMotionMagicConstants.acceleration);//TODO make htis a toggle
				mRodMotor.configMotionCruiseVelocity(RodConstants.outMotionMagicConstants.cruiseVelocity);
				mRodMotor.configMotionSCurveStrength(RodConstants.outMotionMagicConstants.sCurveStrength);
	
	
			} else{
				mRodMotor.selectProfileSlot(1, 0);
	
				mRodMotor.configMotionAcceleration(RodConstants.inMotionMagicConstants.acceleration);
				mRodMotor.configMotionCruiseVelocity(RodConstants.inMotionMagicConstants.cruiseVelocity);
				mRodMotor.configMotionSCurveStrength(RodConstants.inMotionMagicConstants.sCurveStrength);
			}
		}
	
		mRodMotor.set(ControlMode.MotionMagic, (extended? RodConstants.targetOutPosition : RodConstants.targetInPosition), DemandType.ArbitraryFeedForward, RodConstants.kRodGravityFeedforward*getRodAngle().getSin());
	}

	private double rodAngleToSensorPosition(Rotation2d angle){
		return angle.getDegrees()/RodConstants.gearRatio;
	}

	public Rotation2d getRodAngle(){
		return Rotation2d.fromDegrees(mRodMotor.getSelectedSensorPosition()*RodConstants.gearRatio);
	}
	public void setPeriodicOutputsToZero(){
		mPeriodicIO.rollerPercentOutput = 0.0;
		mPeriodicIO.upperConveyorPercentOutput = 0.0;
		mPeriodicIO.lowerConveyorPercentOutput = 0.0;
		mPeriodicIO.shooterPercentOutput = 0.0;
	}
	
/////////////////////////////////////////////////// Misc Methods ///////////////////////////////////////////////////////////////////////////////////////////////////////
	@Override
	public void outputTelemetry() {

	}

	public boolean getRodExtended(){
		return mPeriodicIO.rodExtendedLastLoop;
	}

	public double getRodRawPosition(){
		return mRodMotor.getSelectedSensorPosition();
	}


	public boolean beamBroken(){
		return !mBeamBreak.get();
	}


	public boolean isExtended(){
		return mSolenoid.get();
	}


	public IntakeState getLastState(){
		return mPeriodicIO.lastState;
	}


	public double getCurrentDraw(){
		return mRollerMotor.getOutputCurrent() + mUpperConveyorMotor.getOutputCurrent() + mLowerConveyorMotor.getOutputCurrent() + mShooterMotor.getSupplyCurrent();
	}


	public double getVoltage(){
		return (mRollerMotor.getBusVoltage() + mUpperConveyorMotor.getBusVoltage() + mLowerConveyorMotor.getBusVoltage() + mShooterMotor.getBusVoltage()) / 3.0;
	}
	

	public double getPowerConsumption(){
		return getCurrentDraw()*getVoltage();
	}


    public double getRollerSpeed() {
        return mPeriodicIO.rollerPercentOutput;
    }


	public double getUpperConveyorSpeed(){
		return mPeriodicIO.upperConveyorPercentOutput;
	}


	public double getLowerConveyorSpeed(){
		return mPeriodicIO.lowerConveyorPercentOutput;
	}

	public void zeroRodMotor(){
		mRodMotor.setSelectedSensorPosition(RodConstants.startSensorPosition);
	}

	public void setRodMotorPosition(int position){
		mRodMotor.setSelectedSensorPosition(position);
	}
}