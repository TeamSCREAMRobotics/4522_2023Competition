package frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.team4522.lib.util.TimeBoundIncrementor;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import frc2023.Constants.*;
import frc2023.Constants.IntakeConstants.ConveyorConstants;
import frc2023.Constants.IntakeConstants.LowerConveyorConstants;
import frc2023.Constants.IntakeConstants.ShooterConstants;
import frc2023.Constants.IntakeConstants.UpperConveyorConstants;
import frc2023.PlacementStates.Level;
import frc2023.shuffleboard.tabs.TestTab;

public class Intake extends Subsystem{

	public final PeriodicIO mPeriodicIO = new PeriodicIO();
	private final CANSparkMax mRollerMotor;
	private final Solenoid mSolenoid;
    private final Devices mDevices = Devices.getInstance();
	private final CANSparkMax mUpperConveyorMotor;
	private final CANSparkMax mLowerConveyorMotor;
	private final TalonSRX mShooterMotor;
	private final DigitalInput mBeamBreak;
	
	private Intake(){
		mSolenoid = mDevices.dIntakeSolenoid;
		mRollerMotor = mDevices.dIntakeMotor;
		mUpperConveyorMotor = mDevices.dUpperConveyorMotor;
		mLowerConveyorMotor = mDevices.dLowerConveyorMotor;
        mShooterMotor = mDevices.dShooterMotor;
		mBeamBreak = mDevices.dBeamBreak;
	}
	
	private static Intake mInstance = null;
	public static Intake getInstance(){
		if(mInstance == null){
			mInstance = new Intake();
		}
		return mInstance;
	}

	public static enum IntakeState{
		DISABLED, RETRACT, EXTEND, INTAKE, EJECT, RETRACT_AND_RUN, MANUAL, SHOOT_CUBE_HIGH, SHOOT_CUBE_MID, SHOOT_CUBE_HIGH_AUTO, SHOOT_CUBE_MID_AUTO, 
				BACKWARDS_EJECT, FORCE_RETRACT, SWEEP, REV_FOR_SHOT, INTAKE_FOR_POOPSHOOT, EJECT_ONLY_LOWER_CONVEYOR, 
				AUTO_INTAKE, POOP_SHOOT_FROM_CHARGE_LINE, POOP_SHOOT_FROM_CHARGE_LINE_AUTO, PREPARE_FOR_SHOT;
		
		public boolean isPoopShootState(){
			return this == BACKWARDS_EJECT || this == POOP_SHOOT_FROM_CHARGE_LINE || this == POOP_SHOOT_FROM_CHARGE_LINE_AUTO;
		}
	}

	public class PeriodicIO{
		public double shooterRevPO = 0.0;
		//Inputs
		public IntakeState lastState = IntakeState.RETRACT;
		public double manualRollerPO = 0.0;
		public double manualUpperConveyorPO = 0.0;
		public double manualLowerConveyorPO = 0.0;
		public double manualShooterPO = 0.0;
		public boolean extend = false;
		public TimeBoundIncrementor timeBeforeWheelsCanRun = new TimeBoundIncrementor(0.0, IntakeConstants.kWaitBeforeRunWheelsDuration, IntakeConstants.kWaitBeforeRunWheelsDuration);
		public TimeBoundIncrementor timeBeforeCanShoot = new TimeBoundIncrementor(0.0, ShooterConstants.kTimeBeforeCanShoot, ShooterConstants.kTimeBeforeCanShoot);

		//Outputs
		public IntakeState state = IntakeState.RETRACT;
		public boolean isExtended = false;
		public double rollerPercentOutput = 0.0;
		public double upperConveyorPercentOutput = 0.0;
		public double lowerConveyorPercentOutput = 0.0;
		public double shooterPercentOutput = 0.0;
	}

	public void manual(double rollerPercentOutput, double upperConveyorPercentOutput, double lowerConveyorPercentOutput, double shooterPO, boolean extend){
		mPeriodicIO.state = IntakeState.MANUAL;
		mPeriodicIO.manualRollerPO = rollerPercentOutput;
		mPeriodicIO.manualUpperConveyorPO = upperConveyorPercentOutput;
		mPeriodicIO.manualLowerConveyorPO = lowerConveyorPercentOutput;
		mPeriodicIO.manualShooterPO = shooterPO;
		mPeriodicIO.extend = extend;
	}

	public void sweep(){
		mPeriodicIO.state = IntakeState.SWEEP;
	}

	public void intake(){
		mPeriodicIO.state = IntakeState.INTAKE;
	}

	public void eject(){
		mPeriodicIO.state = IntakeState.EJECT;
	}
	
	public void extend(){
		mPeriodicIO.state = IntakeState.EXTEND;
	}

	public void poopShootFromChargeLine(){
		mPeriodicIO.state = IntakeState.POOP_SHOOT_FROM_CHARGE_LINE;
	}

	
	public void poopShootFromChargeLineAuto(){
		mPeriodicIO.state = IntakeState.POOP_SHOOT_FROM_CHARGE_LINE_AUTO;
	}

	public void autoIntake(){
		mPeriodicIO.state = IntakeState.AUTO_INTAKE;
	}

	@Override
	public void disable(){
		mPeriodicIO.state = IntakeState.DISABLED;
	}

	public void ejectOnlyLowerConveyor(){
		mPeriodicIO.state = IntakeState.EJECT_ONLY_LOWER_CONVEYOR;
	}
	public void backwardsEject(){
		mPeriodicIO.state = IntakeState.BACKWARDS_EJECT;
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

	public void setDesiredState(IntakeState desiredState){
		switch(desiredState){
			case DISABLED:
				disable();
				break;
			case RETRACT:
				retract();
				break;
			case FORCE_RETRACT:
				forceRetract();
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
			case SHOOT_CUBE_HIGH_AUTO:
				shootCubeHighAuto();
				break;
			case SHOOT_CUBE_MID_AUTO:
				shootCubeMidAuto();
				break;
			case BACKWARDS_EJECT:
				backwardsEject();
				break;
			case EJECT_ONLY_LOWER_CONVEYOR:
				ejectOnlyLowerConveyor();
				break;
			case AUTO_INTAKE:
				autoIntake();
				break;
			case POOP_SHOOT_FROM_CHARGE_LINE:
				poopShootFromChargeLine();
				break;
			case INTAKE_FOR_POOPSHOOT:
				intakeForPoopshoot();
				break;
			case POOP_SHOOT_FROM_CHARGE_LINE_AUTO:
				poopShootFromChargeLineAuto();
				break;
			case PREPARE_FOR_SHOT:
				prepareForShot();
				break;
			default:
			DriverStation.reportError("Wrong IntakeState Chosen in setDesiredState()", false);
		}
	}

	public void forceRetract() {
		mPeriodicIO.state = IntakeState.FORCE_RETRACT;
	}

	public void shootCubeMidAuto() {
		mPeriodicIO.state = IntakeState.SHOOT_CUBE_MID_AUTO;
	}

	public void intakeForPoopshoot(){
		mPeriodicIO.state = IntakeState.INTAKE_FOR_POOPSHOOT;
	}

	private void shootCubeHighAuto() {
		mPeriodicIO.state = IntakeState.SHOOT_CUBE_HIGH_AUTO;
	}

	public boolean beamBroken(){
		return !mBeamBreak.get();
	}

	public boolean extended(){
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

	@Override
	public void stop() {
		mPeriodicIO.state = IntakeState.DISABLED;
		mRollerMotor.stopMotor();
		mUpperConveyorMotor.stopMotor();
		mLowerConveyorMotor.stopMotor();
	}

	public void shootCubeHigh(){
		mPeriodicIO.state = IntakeState.SHOOT_CUBE_HIGH;
	}
	
	public void shootCubeMid(){
		mPeriodicIO.state = IntakeState.SHOOT_CUBE_MID;
	}

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
			case FORCE_RETRACT:
				setPeriodicOutputsToZero();
				mPeriodicIO.isExtended = false;
				break;
			case EXTEND:
				setPeriodicOutputsToZero();
				mPeriodicIO.isExtended = true;
				break;
			case INTAKE_FOR_POOPSHOOT://when we intake for poopshoot, we run everything at max because the cube needs to go all the ways in the robot. We also ignore the beam break.
				mPeriodicIO.isExtended = true;
				mPeriodicIO.shooterPercentOutput = 0.0;
				mPeriodicIO.rollerPercentOutput = 1.0;
				mPeriodicIO.upperConveyorPercentOutput = 1.0;
				mPeriodicIO.lowerConveyorPercentOutput = 1.0;
				break;
			case INTAKE:
				mPeriodicIO.isExtended = true;
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
			
				mPeriodicIO.rollerPercentOutput = IntakeConstants.kRetractAndRunPO;
				mPeriodicIO.upperConveyorPercentOutput = UpperConveyorConstants.kRetractAndRunPO;
				mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kRetractAndRunPO;
				mPeriodicIO.shooterPercentOutput = 0.0;

				mPeriodicIO.isExtended = false;
				break;
			case MANUAL:
				mPeriodicIO.rollerPercentOutput = mPeriodicIO.manualRollerPO;
				mPeriodicIO.upperConveyorPercentOutput = mPeriodicIO.manualUpperConveyorPO;
				mPeriodicIO.lowerConveyorPercentOutput = mPeriodicIO.manualLowerConveyorPO;
				mPeriodicIO.shooterPercentOutput = mPeriodicIO.manualShooterPO;
				mPeriodicIO.isExtended = mPeriodicIO.extend;
				break;
			case SHOOT_CUBE_HIGH:
				mPeriodicIO.isExtended = true;

				if(wheelsCanRun){
					// mPeriodicIO.rollerPercentOutput = TestTab.getInstance().shootingSpeedRoller.getDouble(0);//we use these instead if we want to tune values from shuffleboard. In future years, we should have a better tuning setup.
					// mPeriodicIO.upperConveyorPercentOutput = TestTab.getInstance().shootingSpeedTop.getDouble(0);
					// mPeriodicIO.lowerConveyorPercentOutput = TestTab.getInstance().shootingSpeedBottom.getDouble(0);
					// mPeriodicIO.shooterPercentOutput = TestTab.getInstance().shootingSpeedShooter.getDouble(0.0);

					mPeriodicIO.rollerPercentOutput = IntakeConstants.kShootHighPO;
					mPeriodicIO.upperConveyorPercentOutput = UpperConveyorConstants.kShootHighPO;
					mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kShootHighPO;
					mPeriodicIO.shooterPercentOutput = 0.0;
				} else{
					setPeriodicOutputsToZero();
				} 
				break;
			case SHOOT_CUBE_MID:
				mPeriodicIO.isExtended = true;

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
				mPeriodicIO.rollerPercentOutput = IntakeConstants.kSweepPO;
				mPeriodicIO.lowerConveyorPercentOutput = 0.0;
				mPeriodicIO.upperConveyorPercentOutput = 0.0;
				mPeriodicIO.shooterPercentOutput = 0.0;
				break;
			case POOP_SHOOT_FROM_CHARGE_LINE:
				mPeriodicIO.isExtended = true;
				mPeriodicIO.rollerPercentOutput = 1.0;
				mPeriodicIO.upperConveyorPercentOutput = 1.0;
				if(canShoot){
					mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kPoopShootFromChargeLinePO;
				} else{
					mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kPreparePoopShoot;
				}
				mPeriodicIO.shooterPercentOutput = ShooterConstants.kPoopShootFromChargeLinePO;
				break;
			case POOP_SHOOT_FROM_CHARGE_LINE_AUTO:
				mPeriodicIO.isExtended = false;
				mPeriodicIO.rollerPercentOutput = 0.0;
				mPeriodicIO.upperConveyorPercentOutput = 0.0;
				if(canShoot){
					mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kPoopShootFromChargeLineAutoPO;
				} else{
					mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kPreparePoopShoot;
				}
				mPeriodicIO.shooterPercentOutput = ShooterConstants.kPoopShootFromChargeLineAutoPO;
				break;
			case REV_FOR_SHOT:
				mPeriodicIO.isExtended = false;
				mPeriodicIO.rollerPercentOutput = 0.0;
				mPeriodicIO.upperConveyorPercentOutput = 0.0;
				mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kPreparePoopShoot;
				mPeriodicIO.shooterPercentOutput = mPeriodicIO.shooterRevPO;
				break;
			case EJECT_ONLY_LOWER_CONVEYOR:
				mPeriodicIO.isExtended = false;
				mPeriodicIO.rollerPercentOutput = 0.0;
				mPeriodicIO. upperConveyorPercentOutput = 0.0;
				mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kEjectOnlyLowerConveyorPO;
				mPeriodicIO.shooterPercentOutput = 0.0;
				break;
			case AUTO_INTAKE:
				mPeriodicIO.isExtended = true;
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
				mPeriodicIO.rollerPercentOutput = 0.0;
				mPeriodicIO.upperConveyorPercentOutput = 0.0;
				mPeriodicIO.lowerConveyorPercentOutput = LowerConveyorConstants.kPrepareForShot;
				mPeriodicIO.shooterPercentOutput = 0.0;
				break;
		}

		mSolenoid.set(mPeriodicIO.isExtended);
		mRollerMotor.set(mPeriodicIO.rollerPercentOutput);

		mUpperConveyorMotor.set(mPeriodicIO.upperConveyorPercentOutput);
		mLowerConveyorMotor.set(mPeriodicIO.lowerConveyorPercentOutput);
		mShooterMotor.set(ControlMode.PercentOutput, mPeriodicIO.shooterPercentOutput);

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
		lastTimeStamp = timeStamp;
	}

	public void setPeriodicOutputsToZero(){
		mPeriodicIO.rollerPercentOutput = 0.0;
		mPeriodicIO.upperConveyorPercentOutput = 0.0;
		mPeriodicIO.lowerConveyorPercentOutput = 0.0;
		mPeriodicIO.shooterPercentOutput = 0.0;
	}
	
	@Override
	public void outputTelemetry() {
		
	}

    public boolean isExtended() {
        return mPeriodicIO.isExtended;
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

    public void revForShot(double revPO) {
		mPeriodicIO.state = IntakeState.REV_FOR_SHOT;
		mPeriodicIO.shooterRevPO = revPO;
    }

	public void prepareForShot() {
		mPeriodicIO.state = IntakeState.PREPARE_FOR_SHOT;
	}
}