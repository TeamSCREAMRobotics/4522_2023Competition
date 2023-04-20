package frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.MagnetFieldStrength;
import com.team4522.lib.deviceConfiguration.DeviceUtil;
import com.team4522.lib.pid.PIDConstants;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc2023.Constants.ArmConstants;
import frc2023.PlacementStates.Level;

    /**
     * The pivot has slop, which means that the encoder reading of the motor will be different from the position of the arm by a value equal to half of the total slop.
     * Our code uses the convention of Arm position is the actual arm, and sprocket position is the position that the motor goes to. For our setpoints, we use the location
     * of the arm as the target, but to get the arm there we have to convert the arm position to the sprocket position based on the slop 
     * of the pivot(which changes over time due to design modifications, chain stretching, etc. )
     * 
     * <p> We also have two sensors on the arm. One is the integrated sensor on our pivot TalonFX, which is able to measure the sprocketAngle. The other sensor is a 
     * CANCoder that is mounted on the actual arm axle, which can only measure the arm position. Based on our measured slop value, we can convert between the two. We don't
     * use the CANCoder anymore because it was mounted poorly, and we zero the arm with a level before every match.
     */
public class Arm extends Subsystem{

    private static Arm mInstance = null;
	public static Arm getInstance(){
		if(mInstance == null){
			mInstance = new Arm();
		}
		return mInstance;
	}
   
	public static enum ArmState{
		DISABLED, MANUAL, POSITION, RETRIEVE_CUBE, RETRIEVE_CONE, PREPARE_PLACEMENT, POSITION_NO_LIMITS, POOP_SHOOT, CONE_HOLD, PLACE_HYBRID_CONE, PLACE_MID_CONE, PLACE_TOP_CONE
	}

	public static class PeriodicIO{
        //Inputs
        public ArmState lastState = ArmState.DISABLED;

        //Outputs
		public ArmState state;
        public Rotation2d targetSprocketAngle;
        public double targetLength;
        public double pivotPercentOutput;
        public double telescopePercentOutput;
        public boolean hasCone;
	}
    public final PeriodicIO mPeriodicIO = new PeriodicIO();

    private final TalonFX mPivotMotor;
    private final CANCoder mPivotEncoder;
    private final TalonSRX mTelescopeMotor;
    private final Devices mDevices = Devices.getInstance();
	/**mutliply this by native steer native position to get degrees */
	public static final double kPivotPositionCoefficient = 1.0 / ArmConstants.kPivotTicksPerRotation / ArmConstants.kPivotGearRatio * 360.0;
	/**mutliply this by steer native velocity to get degrees per second */
	public static final double kPivotVelocityCoefficient = kPivotPositionCoefficient * 10.0;

    public static final double kTelescopePositionCoefficient = (ArmConstants.kMaxTelescopeLength - ArmConstants.kMinTelescopeLength) / (ArmConstants.kPotentiometerMax - ArmConstants.kPotentiometerMin);
    public static final double kTelescopeVelocityCoefficient = kTelescopePositionCoefficient * 10.0;

    public Arm(){
        mPivotMotor = mDevices.dArmPivot;
        mTelescopeMotor = mDevices.dArmTelescope;
        mPivotEncoder = mDevices.dPivotCANCoder;

        DeviceUtil.configTalonFXPID(mPivotMotor, ArmConstants.pivotPIDConstants, true);
        DeviceUtil.configTalonSRXPID(mTelescopeMotor, ArmConstants.telescopePIDConstants, true);

        DeviceUtil.configTalonFXMotionMagic(mPivotMotor, ArmConstants.pivotMotionMagicConstants, true);
        DeviceUtil.configTalonSRXMotionMagic(mTelescopeMotor, ArmConstants.telescopeMotionMagicConstants, true);
    }


    //We have two sensors that can read the pivot. We have the onboard sensor for the TalonFX and a CANCoder mounted on the pivot. The CANCoder is mounted poorly, so we don't use it anymore
    public static enum PivotSensorType{
        CANCODER, INTEGRATED_SENSOR
    }


    public void resetPivotToAngle(Rotation2d armAngle) {         
		mPivotMotor.setSelectedSensorPosition(armPositionToSprocketPosition(armAngle).getDegrees()/kPivotPositionCoefficient);
    }

        
    public void zeroTelescope(){
        mTelescopeMotor.setSelectedSensorPosition(ArmConstants.kPotentiometerMin);
    }


    public Rotation2d getArmAngle(PivotSensorType sensor){
        if(sensor == PivotSensorType.CANCODER) return getAbsoluteEncoder();

        Rotation2d sensorAngle = nativePivotPositionToRotation(mPivotMotor.getSelectedSensorPosition());
        if(sensorAngle.getDegrees() > 0) return addSlop(sensorAngle);
        else return subtractSlop(sensorAngle);
    }

    
    public Rotation2d getSprocketAngle(PivotSensorType sensor){
        if(sensor == PivotSensorType.INTEGRATED_SENSOR) return nativePivotPositionToRotation(mPivotMotor.getSelectedSensorPosition());

        Rotation2d cancoderAngle = getAbsoluteEncoder();

        if(cancoderAngle.getDegrees() > 0) return subtractSlop(cancoderAngle);
        else return addSlop(cancoderAngle);
    }

    /////////////////////////////////////// Methods to set the arm state //////////////////////////////////////////////////////////////////////////////////

    public void setPercentOutput(double pivotOutput, double telescopeOutput){
        mPeriodicIO.state = ArmState.MANUAL;
        mPeriodicIO.pivotPercentOutput = pivotOutput;
        mPeriodicIO.telescopePercentOutput = telescopeOutput;
    }


    public void setPosition(Translation2d armPosition, boolean hasCone){
        mPeriodicIO.hasCone = hasCone;
        mPeriodicIO.state = ArmState.POSITION;
        updateTargetSetpoint(armPosition);
    }


    public void setPosition(double armLength, Rotation2d pivotRotation, boolean hasCone){
        setPosition(new Translation2d(armLength, pivotRotation), hasCone);
    }


    /**
     * Runs like the normal position mode, but we don't suck the telescope in while the arm is pivoting. This is what we use in auto when we slam the cone down.
     */
    public void setPositionWithoutLimits(Translation2d armPosition, boolean hasCone){
        mPeriodicIO.hasCone = hasCone;
        mPeriodicIO.state = ArmState.POSITION_NO_LIMITS;
        updateTargetSetpoint(armPosition);
    }


    public void placeHybridCone(){
        mPeriodicIO.hasCone = true;
        mPeriodicIO.state = ArmState.PLACE_HYBRID_CONE;
        updateTargetSetpoint(ArmConstants.Setpoints.kHybridCone);
    }

    
    public void placeMidCone(){
        mPeriodicIO.hasCone = true;
        mPeriodicIO.state = ArmState.PLACE_MID_CONE;
        updateTargetSetpoint(ArmConstants.Setpoints.kMiddleCone);
    }

    
    public void placeTopCone(){
        mPeriodicIO.hasCone = true;
        mPeriodicIO.state = ArmState.PLACE_TOP_CONE;
        updateTargetSetpoint(ArmConstants.Setpoints.kTopCone);
    }


    public void setPlacementPosition(Level level){
        switch(level){
            case HYBRID:
                placeHybridCone();
                break;
            case MIDDLE:
                placeMidCone();
                break;
            case TOP:
                placeTopCone();
                break;
        }
    }


    public void setConeHold() {
        mPeriodicIO.hasCone = true;
        mPeriodicIO.state = ArmState.CONE_HOLD;
        updateTargetSetpoint(ArmConstants.Setpoints.kConeHold);
    }


    public void retrieveCube(){
        mPeriodicIO.hasCone = false;
        mPeriodicIO.state = ArmState.RETRIEVE_CUBE;
        updateTargetSetpoint(ArmConstants.Setpoints.kCubeRetrieval);
    }


    public void retrieveCone(){
        mPeriodicIO.hasCone = false;
        mPeriodicIO.state = ArmState.RETRIEVE_CONE;
        updateTargetSetpoint(ArmConstants.Setpoints.kConeRetrieval);
    }


    public void preparePlacement(boolean hasCone){
        mPeriodicIO.hasCone = hasCone;
        mPeriodicIO.state = ArmState.PREPARE_PLACEMENT;
        updateTargetSetpoint(ArmConstants.Setpoints.kPreparePlacement);
    }


    public void setPoopShootPosition() {
        mPeriodicIO.state = ArmState.POOP_SHOOT;
        updateTargetSetpoint(ArmConstants.Setpoints.kPoopShoot);
    }


    private void updateTargetSetpoint(Translation2d setpoint){
        mPeriodicIO.targetSprocketAngle = armPositionToSprocketPosition(setpoint.getAngle());
        mPeriodicIO.targetLength = boundTelescopeLength(setpoint.getNorm());
    }


    @Override
    public void stop() {
        mPeriodicIO.state = ArmState.DISABLED;
        mPivotMotor.set(ControlMode.PercentOutput, 0.0);
        mTelescopeMotor.set(ControlMode.PercentOutput, 0);  
    }


    @Override
    public void disable() {
        mPeriodicIO.state = ArmState.DISABLED;
    }

    //////////////////////////////////////////// Writing Outputs ///////////////////////////////////////////////////////////////

    public double calculatePivotGravityFeedforward(boolean hasCone){
        return -getArmAngle(PivotSensorType.INTEGRATED_SENSOR).getSin() * (hasCone? ArmConstants.gravityFeedforwardMapWithCone.get(getLength()) : ArmConstants.gravityFeedforwardMapWithoutCone.get(getLength()));
    }


    public double calculateTelescopeGravityFeedforward(boolean hasCone, double pivotError){ 
        return  Math.signum(pivotError) * (hasCone? ArmConstants.telescopeKSWithCone : ArmConstants.telescopeKSWithoutCone)  +  getArmAngle(PivotSensorType.INTEGRATED_SENSOR).getCos() * (hasCone? ArmConstants.telescopeKFGravityWithCone : ArmConstants.telescopeKFGravityWithoutCone);
    }


    private void suckInTelescope(){
        if(!telescopeOnTarget(ArmConstants.kSuckInLength)){
            mTelescopeMotor.set(ControlMode.MotionMagic, armLengthToNativeUnits(ArmConstants.kSuckInLength), DemandType.ArbitraryFeedForward, calculateTelescopeGravityFeedforward(mPeriodicIO.hasCone, (ArmConstants.kSuckInLength-getLength())));
        } 
        else mTelescopeMotor.set(ControlMode.PercentOutput, 0);
    }
    

    private void moveTelescopeToTarget(double targetLength, boolean hasCone){
        if(!telescopeOnTarget(targetLength)) mTelescopeMotor.set(ControlMode.MotionMagic, armLengthToNativeUnits(targetLength), DemandType.ArbitraryFeedForward, calculateTelescopeGravityFeedforward(hasCone, getTelescopeLengthError(targetLength)));
        else mTelescopeMotor.set(ControlMode.PercentOutput, 0);
    }


    private void movePivotToTarget(Rotation2d targetSprocketAngle, boolean hasCone){
        if(pivotOnTarget(targetSprocketAngle)){
            mPivotMotor.set(ControlMode.PercentOutput, 0);
        } else{
            mPivotMotor.set(ControlMode.MotionMagic, pivotAngleToNativePosition(targetSprocketAngle), DemandType.ArbitraryFeedForward, calculatePivotGravityFeedforward(hasCone));
        }
    }


    @Override
    public void writeOutputs() {
        switch(mPeriodicIO.state){
            case DISABLED:
                mPivotMotor.set(ControlMode.PercentOutput, 0);
                mTelescopeMotor.set(ControlMode.PercentOutput, 0);
                break;
            case MANUAL:
                mPivotMotor.set(ControlMode.PercentOutput, mPeriodicIO.pivotPercentOutput, DemandType.ArbitraryFeedForward, 0);
                mTelescopeMotor.set(ControlMode.PercentOutput, mPeriodicIO.telescopePercentOutput, DemandType.ArbitraryFeedForward, 0);
                break;
            case PLACE_MID_CONE:
                if(pivotOnThresholdForTelescopeOut(mPeriodicIO.targetSprocketAngle)){
                    moveTelescopeToTarget(mPeriodicIO.targetLength, mPeriodicIO.hasCone);
                } else{
                    suckInTelescope();
                }

                if(getLength() > (ArmConstants.Setpoints.kMiddleCone.getNorm()+0.1)){
                    mPivotMotor.set(ControlMode.PercentOutput, 0);
                } else{
                    movePivotToTarget(mPeriodicIO.targetSprocketAngle, mPeriodicIO.hasCone);
                }

                break;
            case PREPARE_PLACEMENT:
            case POOP_SHOOT:
            case PLACE_HYBRID_CONE:
            case PLACE_TOP_CONE:
            case POSITION:
            // System.out.println("((((()))))");
            
                if(pivotOnThresholdForTelescopeOut(mPeriodicIO.targetSprocketAngle)){
                    moveTelescopeToTarget(mPeriodicIO.targetLength, mPeriodicIO.hasCone);
                } else{
                    suckInTelescope();
                }
                movePivotToTarget(mPeriodicIO.targetSprocketAngle, mPeriodicIO.hasCone);

                break;
            case RETRIEVE_CUBE:
            case POSITION_NO_LIMITS:
            case CONE_HOLD:
            case RETRIEVE_CONE:
                moveTelescopeToTarget(mPeriodicIO.targetLength, mPeriodicIO.hasCone);
                // System.out.println("&^^^^^&&&&");
                movePivotToTarget(mPeriodicIO.targetSprocketAngle, mPeriodicIO.hasCone);
                break;
            default:
                break;
        }
        mPeriodicIO.lastState = mPeriodicIO.state;
    }

///////////////////////////////////////// Arm Position Logic Methods /////////////////////////////////////////////////////////////////////

    private static Rotation2d armPositionToSprocketPosition(Rotation2d armPosition){// all of this logic will fail if the arm is within the wiggle room of being completely vertical. We cannot control the arm if gravity doesn't hold it to one side of its wiggle
        if(armPosition.getDegrees() > 0) return subtractSlop(armPosition);
        else return addSlop(armPosition);
    }


    private static Rotation2d sprocketPositionToArmPosition(Rotation2d sprocketPosition){
        if(sprocketPosition.getDegrees() > 0) return addSlop(sprocketPosition);
        else return subtractSlop(sprocketPosition);
    }
    

    private static Rotation2d addSlop(Rotation2d rotation){
        return rotation.plus(ArmConstants.kArmSlop.div(2));
    }


    private static Rotation2d subtractSlop(Rotation2d rotation){
        return rotation.minus(ArmConstants.kArmSlop.div(2));
    }


    private static Rotation2d nativePivotPositionToRotation(double nativePosition){
        return Rotation2d.fromDegrees(nativePosition * kPivotPositionCoefficient);
    }


    private static double pivotAngleToNativePosition(Rotation2d pivotAngle){
        return pivotAngle.getDegrees() / kPivotPositionCoefficient;
    }


    public static double nativeUnitsToArmLength(double nativeUnits){
        return ArmConstants.kMinTelescopeLength + (nativeUnits - ArmConstants.kPotentiometerMin) * kTelescopePositionCoefficient;
    }


    public static double armLengthToNativeUnits(double armLength){
        return (armLength - ArmConstants.kMinTelescopeLength) / kTelescopePositionCoefficient + ArmConstants.kPotentiometerMin;
    }   


    private double boundTelescopeLength(double desiredLength){
        return MathUtil.clamp(desiredLength, ArmConstants.kMinTelescopeLength, ArmConstants.kMaxTelescopeLength);
    }


    public boolean atTargetPosition(Rotation2d pivotTarget, double telescopeTarget){
        return pivotOnTarget(pivotTarget) && telescopeOnTarget(telescopeTarget);
    }   


    public boolean atTargetPosition(Translation2d position){
        return atTargetPosition(position.getAngle(), position.getNorm());
    }
    

    public boolean pivotOnTarget(Rotation2d pivotTarget){
		return Math.abs(getPivotError(pivotTarget).getDegrees()) < ArmConstants.kPivotAngleOnTargetThreshold.getDegrees();
	}


    /**
     * If the arm is within a certain threshold of our target position, this is true, otherwise we suck in the telescope while pivoting
     */
    public boolean pivotOnThresholdForTelescopeOut(Rotation2d pivotTarget){
		return Math.abs(getPivotError(pivotTarget).getDegrees()) < ArmConstants.kPivotAngleThresholdForTelescopeOut.getDegrees();
    }


    public boolean telescopeOnTarget(double targetLength){
		return Math.abs(getTelescopeLengthError(targetLength)) < ArmConstants.kTelescopeAtTargetThreshold;
	}

////////////////////////////////////////////////////////// Misc Methods ////////////////////////////////////////////////////////////////////

    @Override
    public void outputTelemetry(){}


    public void setNeutralMode(NeutralMode pivotMode, NeutralMode telescopeMode) {
        mPivotMotor.setNeutralMode(pivotMode);
        mTelescopeMotor.setNeutralMode(telescopeMode);
    }


    public void incrementSensorReading(Rotation2d addition) {
        resetPivotToAngle(getArmAngle(PivotSensorType.INTEGRATED_SENSOR).plus(addition));
    }


    public void configTelescopeSoftLimitsEnabled(boolean enabled){
        //These methods could be used with our error checker, but we don't want to use it here. The error checker uses a while loop, so if the device didn't configure properly, 
            // the robot could lose control for up to a few seconds. If the arm has a connection issue, we will still see it before the match when it configures, and 
            //if it fails mid-match we can't do anything about a bad wire connection and will have to use manual override anyways
        mTelescopeMotor.configReverseSoftLimitEnable(enabled);
        mTelescopeMotor.configForwardSoftLimitEnable(enabled);
    }


    public void configPivotSoftLimitsEnabled(boolean enabled){
        mPivotMotor.configForwardSoftLimitEnable(enabled);
        mPivotMotor.configReverseSoftLimitEnable(enabled);
    }


    public void updatePivotPIDConstants(PIDConstants constants) {
        DeviceUtil.configTalonFXPID(mPivotMotor, constants, false);
    }


    public void updateTelescopePIDConstants(PIDConstants constants) {
        DeviceUtil.configTalonSRXPID(mTelescopeMotor, constants, false);
    }

////////////////////////////// Getters ///////////////////////////////////////////////////////////////////////////

    public ArmState getLastState(){
        return mPeriodicIO.lastState;
    }


    public ArmState getDesiredState(){
        return mPeriodicIO.state;
    }


    public double getLength(){
        return nativeUnitsToArmLength(mTelescopeMotor.getSelectedSensorPosition());
    }


    public double getDesiredLength(){
        return mPeriodicIO.targetLength;
    }


    public Rotation2d getPivotTarget(){
        return mPeriodicIO.targetSprocketAngle;
    }

    
    public double getTelescopeSensorPosition() {
        return mTelescopeMotor.getSelectedSensorPosition();
    }


    public double getPivotSensorPosition() {
        return mPivotMotor.getSelectedSensorPosition();
    }


    public double getPivotSensorVelocity() {
        return mPivotMotor.getSelectedSensorVelocity();
    }


    public Rotation2d getPivotError(Rotation2d pivotTarget){
        return pivotTarget.minus(getArmAngle(PivotSensorType.INTEGRATED_SENSOR));
    }


    public double getTelescopeLengthError(double targetLength){
        return targetLength - getLength();
    }
    

    private Rotation2d getAbsoluteEncoder(){
        return ScreamUtil.boundRotation(Rotation2d.fromDegrees(mPivotEncoder.getAbsolutePosition()).minus(ArmConstants.PivotAngleOffset));
    }
    

    public MagnetFieldStrength getPivotMagnetStatus() {
        return mPivotEncoder.getMagnetFieldStrength();
    }


	public double getCurrentDraw(){
		return mPivotMotor.getSupplyCurrent() + mTelescopeMotor.getSupplyCurrent();
	}


	public double getVoltage(){
		return (mPivotMotor.getBusVoltage() + mTelescopeMotor.getBusVoltage()) / 2;
	}


	public double getPowerConsumption(){
		return mPivotMotor.getSupplyCurrent()*mPivotMotor.getBusVoltage() + mTelescopeMotor.getSupplyCurrent()*mTelescopeMotor.getBusVoltage();
	}
}