package frc2023.shuffleboard.tabs;

import com.team4522.lib.pid.PIDConstants;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc2023.Constants;
import frc2023.Constants.ArmConstants;
import frc2023.shuffleboard.ShuffleboardTabBase;
import frc2023.subsystems.Arm;
import frc2023.subsystems.Arm.ArmState;
import frc2023.subsystems.Arm.PivotSensorType;

public class ArmTab extends ShuffleboardTabBase{

    private GenericPublisher mLastArmState;
    private GenericPublisher mDemandedArmState;
   
	private GenericPublisher mDesiredPivotAngle;
	private GenericPublisher mSprocketAngleSensor;
	private GenericPublisher mSprocketAngleCANCoder;
    private GenericPublisher mArmPositionSensor;
    private GenericPublisher mArmPositionCANCoder;

    private GenericPublisher mDesiredTelescopeLength;
    private GenericPublisher mMeasuredTelescopeLength;

	private GenericPublisher mCurrentDraw;
	private GenericPublisher mVoltage;
	private GenericPublisher mPowerConsumption;
    

    private GenericPublisher mTelescopeSensorPosition;
    
    
    private GenericEntry mPivotKP;
    private GenericEntry mPivotKD;
    
    private GenericEntry mTelescopeKP;
    private GenericEntry mTelescopeKD;


    private final Arm mArm = Arm.getInstance();

	private static ArmTab mInstance = null;
	public static ArmTab getInstance(){
		if(mInstance == null){
			mInstance = new ArmTab();
		}
		return mInstance;
	}

	@Override
	public void createEntries() {
		mTab = Shuffleboard.getTab("ArmTab");

        mDemandedArmState = createStringEntry("Demanded Arm State", ArmState.DISABLED.toString());
        mLastArmState = createStringEntry("Last Arm State", ArmState.DISABLED.toString());

        mTelescopeSensorPosition = createNumberEntry("TelescopeSensorPosition", -1);

        mDesiredPivotAngle = createNumberEntry("DesiredPivotAngle (Deg)", 0);
        mDesiredTelescopeLength = createNumberEntry("Desired Telescope Length (meters)", 0);
        mMeasuredTelescopeLength = createNumberEntry("Measured Telescope Length (meters)", 0);
        mSprocketAngleSensor = createNumberEntry("SprocketAngleSensor (Deg)", 0);
        mSprocketAngleCANCoder = createNumberEntry("SprocketAngleCANCoder (Deg)", 0);
        mArmPositionSensor = createNumberEntry("ArmPositionSensor", 0);
        mArmPositionCANCoder = createEntry("Arm Position CANCoder", 0.0);

        mCurrentDraw = createNumberEntry("Current Draw (A)", 0.0);
        mVoltage = createNumberEntry("Voltage (V)", 0.0);
        mPowerConsumption = createNumberEntry("Power Consumption (W)", 0.0);

		if(Constants.updatePIDsFromShuffleboard){
			mPivotKP = createNumberEntry("Pivot kP", ArmConstants.pivotPIDConstants.kP());
			mPivotKD = createNumberEntry("Pivot kD", ArmConstants.pivotPIDConstants.kD());
	
			mTelescopeKP = createNumberEntry("Telescope kP", ArmConstants.telescopePIDConstants.kP());
			mTelescopeKD = createNumberEntry("Telescope kD", ArmConstants.telescopePIDConstants.kD());
		}
	}

	@Override
	public void update() {
        try{
        mDesiredPivotAngle.setDouble(mArm.getPivotTarget().getDegrees());
        mDemandedArmState.setString(mArm.getDesiredState().toString());
        mDesiredTelescopeLength.setDouble(mArm.getDesiredLength());

                    
        } catch(Exception e){

        }
        mLastArmState.setString(mArm.getLastState().toString());
        mSprocketAngleSensor.setDouble(mArm.getSprocketAngle(PivotSensorType.INTEGRATED_SENSOR).getDegrees());
        mSprocketAngleCANCoder.setDouble(mArm.getSprocketAngle(PivotSensorType.CANCODER).getDegrees());
        mArmPositionSensor.setDouble(mArm.getArmAngle(PivotSensorType.INTEGRATED_SENSOR).getDegrees());
        mArmPositionCANCoder.setDouble(mArm.getArmAngle(PivotSensorType.CANCODER).getDegrees());
        mMeasuredTelescopeLength.setDouble(mArm.getLength());
        mTelescopeSensorPosition.setDouble(mArm.getTelescopeSensorPosition());
        
		mCurrentDraw.setDouble(mArm.getCurrentDraw());
		mVoltage.setDouble(mArm.getVoltage());
		mPowerConsumption.setDouble(mArm.getPowerConsumption());
	}

    
    public void updatePivotPIDConstants(){
        PIDConstants constants = new PIDConstants();
        constants.setkP(mPivotKP.getDouble(ArmConstants.pivotPIDConstants.kP()));
        constants.setkD(mPivotKD.getDouble(ArmConstants.pivotPIDConstants.kD()));
		mArm.updatePivotPIDConstants(constants);
    }

    public void updateTelescopePIDConstants(){
        PIDConstants constants = new PIDConstants();
        constants.setkP(mTelescopeKP.getDouble(ArmConstants.telescopePIDConstants.kP()));
        constants.setkD(mTelescopeKD.getDouble(ArmConstants.telescopePIDConstants.kD()));
		mArm.updateTelescopePIDConstants(constants);
    }
}