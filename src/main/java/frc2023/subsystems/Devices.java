package frc2023.subsystems;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.team254.CanDeviceId;

import com.team4522.lib.drivers.DeviceConfigurationUtil;
import com.team4522.lib.drivers.DeviceConfigs.CANCoderConfig;
import com.team4522.lib.drivers.DeviceConfigs.CANSparkMaxConfig;
import com.team4522.lib.drivers.DeviceConfigs.Pigeon2Config;
import com.team4522.lib.drivers.DeviceConfigs.TalonFXConfig;
import com.team4522.lib.drivers.DeviceConfigs.TalonSRXConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import frc2023.Constants;
import frc2023.Ports;
import frc2023.Constants.ArmConstants;
import frc2023.Constants.SwerveConstants;

/** This class stores every device that the robot uses. The purpose of this class is to initialize everything on startup and run our device checking system to make sure
 *  that everything is created properly. This class is referenced in every subsystem to dependency inject our devices. Because of this, this class must be called before any of the
 *  other classes in our code.
 * @see frc2023.Robot#Robot()
 */
public class Devices {
    
    private static Devices mInstance = null;
	public static Devices getInstance(){
		if(mInstance == null){
			mInstance = new Devices();
		}
		return mInstance;
	}

    public final TalonFX dFLDrive;//d is for device.
    public final TalonFX dFLSteer;
    public final CANCoder dFLCancoder;
    
    public final TalonFX dBLDrive;
    public final TalonFX dBLSteer;
    public final CANCoder dBLCancoder;
    
    public final TalonFX dBRDrive;
    public final TalonFX dBRSteer;
    public final CANCoder dBRCancoder;
    
    public final TalonFX dFRDrive;
    public final TalonFX dFRSteer;
    public final CANCoder dFRCancoder;

    public final Pigeon2 dGyro;
    public final TalonFX dArmPivot;
    public final CANCoder dPivotCANCoder;
    public final TalonSRX dArmTelescope;
    public final Solenoid dUpperGripperSolenoid;
    public final Solenoid dLowerGripperSolenoid;

    public final CANSparkMax dIntakeMotor;
    public final Solenoid dIntakeSolenoid;
    public final CANSparkMax dUpperConveyorMotor;
    public final CANSparkMax dLowerConveyorMotor;
    
    public final DigitalInput dBeamBreak;
    public final edu.wpi.first.wpilibj.Compressor dCompressor;
    public final TalonSRX dShooterMotor;

    private Devices(){
        dGyro = createGyro(Ports.pigeonID);
        dCompressor  = new edu.wpi.first.wpilibj.Compressor(Ports.pneumaticsHubID, PneumaticsModuleType.REVPH);

        dFLDrive = createSwerveModuleDriveMotor(SwerveConstants.FLConstants.driveID);
        dFLSteer = createSwerveModuleSteerMotor(SwerveConstants.FLConstants.steerId);
        dFLCancoder = createSwerveModuleCANCoder(SwerveConstants.FLConstants.encoderID);

        dBLDrive = createSwerveModuleDriveMotor(SwerveConstants.BLConstants.driveID);
        dBLSteer = createSwerveModuleSteerMotor(SwerveConstants.BLConstants.steerId);
        dBLCancoder = createSwerveModuleCANCoder(SwerveConstants.BLConstants.encoderID);
        
        dBRDrive = createSwerveModuleDriveMotor(SwerveConstants.BRConstants.driveID);
        dBRSteer = createSwerveModuleSteerMotor(SwerveConstants.BRConstants.steerId);
        dBRCancoder = createSwerveModuleCANCoder(SwerveConstants.BRConstants.encoderID);
        
        dFRDrive = createSwerveModuleDriveMotor(SwerveConstants.FRConstants.driveID);
        dFRSteer = createSwerveModuleSteerMotor(SwerveConstants.FRConstants.steerId);
        dFRCancoder = createSwerveModuleCANCoder(SwerveConstants.FRConstants.encoderID);

        dPivotCANCoder = createPivotCANCoder(Ports.ArmPivotEncoderID);
        dArmPivot = createArmPivotMotor(Ports.ArmPivotID);
        dArmTelescope = createArmTelecopeMotor(Ports.ArmTelescopeID);

        dUpperGripperSolenoid = new Solenoid(Ports.pneumaticsHubID, PneumaticsModuleType.REVPH, Ports.gripperUpperSolenoidID);
        dLowerGripperSolenoid = new Solenoid(Ports.pneumaticsHubID, PneumaticsModuleType.REVPH, Ports.gripperLowerSolenoidID);

        dIntakeSolenoid = new Solenoid(Ports.pneumaticsHubID, PneumaticsModuleType.REVPH, Ports.intakeSolenoidID);
        dIntakeMotor = createIntakeMotor(Ports.intakeMotorID);
        dUpperConveyorMotor = createUpperConveyorMotor(Ports.upperConveyorID);
        dLowerConveyorMotor = createLowerConveyorMotor(Ports.lowerConveyorID);
        dShooterMotor = createShooterMotor(Ports.shooterID);
        dBeamBreak = new DigitalInput(Ports.beamBreakID);
    }

	public static final int maxStatusFramePeriod = 255;

    private static TalonFX createSwerveModuleDriveMotor(CanDeviceId id){
        TalonFXConfig config = new TalonFXConfig();
        config.status_13_Base_PIDF0 = 20;
        config.status_2_Feedback0 = Constants.kOdometryPeriodMilliseconds / 2;
        config.feedbackDevice = FeedbackDevice.IntegratedSensor;

        TalonFX drive = DeviceConfigurationUtil.configTalonFX(new TalonFX(id.getDeviceNumber(), id.getBus()), config, "SwerveModuleDriveMotor "  + id.getDeviceNumber() + "  ");

        drive.setNeutralMode(NeutralMode.Brake);
        drive.setInverted(TalonFXInvertType.Clockwise);
		drive.setSensorPhase(true);

		return drive;
    }

    /**
     * Creates a TalonFX swerve module steer motor and confiures settings that are not constants for the swerve module.
     * @param id
     * @return
     */
    private static TalonFX createSwerveModuleSteerMotor(CanDeviceId id){
        
        TalonFXConfig config = new TalonFXConfig();
        config.status_10_Targets = 20;
        config.status_13_Base_PIDF0 = 20;
        config.status_2_Feedback0 = Constants.kOdometryPeriodMilliseconds;
        config.feedbackDevice = FeedbackDevice.IntegratedSensor;
        config.statorCurrentLimitConfiguration = new StatorCurrentLimitConfiguration(false, 20, 25, .1);//idk if we will ever use this
        TalonFX steer = DeviceConfigurationUtil.configTalonFX(new TalonFX(id.getDeviceNumber(), id.getBus()), config, "SwerveModuleSteerMotor "  + id.getDeviceNumber() + "  ");
            
        steer.setNeutralMode(NeutralMode.Brake);
        steer.setInverted(true);//TODO make consistent with clockwise
        steer.setSensorPhase(true);

        return steer;
    }

	private static CANCoder createSwerveModuleCANCoder(CanDeviceId id){
		
        CANCoderConfig config = new CANCoderConfig();
        config.sensorDataPeriod = 100;
        config.vBatAndFaultsPeriod = maxStatusFramePeriod;
		CANCoder encoder = DeviceConfigurationUtil.configCANCoder(new CANCoder(id.getDeviceNumber(), id.getBus()), config, "SwerveModuleCANCoder "  + id.getDeviceNumber() + "  ");

		return encoder;
	}

    public static Pigeon2 createGyro(CanDeviceId id){

        Pigeon2Config config = new Pigeon2Config();
        config.condStatus_9_SixDeg_YPR = Constants.kOdometryPeriodMilliseconds/2;
        Pigeon2 gyro = DeviceConfigurationUtil.configPigeon2(new Pigeon2(id.getDeviceNumber(), id.getBus()), config, "Pigeon2 "  + id.getDeviceNumber() + "  ");
        return gyro;
    }

    public static TalonFX createArmPivotMotor(CanDeviceId id){

        TalonFXConfig config = new TalonFXConfig();
        config.status_2_Feedback0 = 20;
        config.feedbackDevice = FeedbackDevice.IntegratedSensor;
        config.status_13_Base_PIDF0 = 20;
        config.status_10_Targets = 20;
        config.voltageCompSaturation = 8;
        config.forwardSoftLimit = ArmConstants.pivotForwardSoftLimit;
        config.reverseSoftLimit = ArmConstants.pivotReverseSoftLimit;

        TalonFX pivotMotor = DeviceConfigurationUtil.configTalonFX(new TalonFX(id.getDeviceNumber(), id.getBus()), config, "PivotMotor "  + id.getDeviceNumber() + "  ");
        
        pivotMotor.setNeutralMode(NeutralMode.Brake);
        pivotMotor.enableVoltageCompensation(true);
        pivotMotor.setInverted(false);
        pivotMotor.setSensorPhase(false);

        return pivotMotor;
    }

    public static TalonSRX createArmTelecopeMotor(int id){
        
        TalonSRXConfig config = new TalonSRXConfig();
        config.feedbackDevice = FeedbackDevice.QuadEncoder;
        config.status_10_Targets = 20;
        config.status_13_Base_PIDF0 = 20;
        config.status_2_Feedback0 = 20;
        config.forwardSoftLimit = ArmConstants.kPotentiometerMax - ArmConstants.kDistanceForSoftLimit;
        config.reverseSoftLimit = ArmConstants.kPotentiometerMin + ArmConstants.kDistanceForSoftLimit;
        config.voltageCompSaturation = 10.0;

        TalonSRX telescopeMotor = DeviceConfigurationUtil.configTalonSRX(new TalonSRX(id), config, "TelescopeMotor " + id + "  ");

        telescopeMotor.setInverted(false);
        telescopeMotor.setSensorPhase(false);
        
        telescopeMotor.enableVoltageCompensation(true);
        telescopeMotor.setNeutralMode(NeutralMode.Brake);
        return telescopeMotor;
    }


    public static CANSparkMax createLowerConveyorMotor(int id){
        CANSparkMaxConfig config = new CANSparkMaxConfig();
        config.idleMode = IdleMode.kBrake;
        CANSparkMax motor = DeviceConfigurationUtil.configCANSparkMax(new CANSparkMax(id, MotorType.kBrushless), config, "LowerConveyor " + id + "  ");
        motor.setInverted(false);
        return motor;
    }

    public static CANSparkMax createUpperConveyorMotor(int id){
        CANSparkMaxConfig config = new CANSparkMaxConfig();
        config.idleMode = IdleMode.kBrake;
        CANSparkMax motor = DeviceConfigurationUtil.configCANSparkMax(new CANSparkMax(id, MotorType.kBrushless), config, "UpperConveyor "  + id + "  ");
        motor.setInverted(true);
        return motor;
    }

    public static CANSparkMax createIntakeMotor(int id){
        CANSparkMaxConfig config = new CANSparkMaxConfig();
        config.idleMode = IdleMode.kBrake;
        CANSparkMax motor = DeviceConfigurationUtil.configCANSparkMax(new CANSparkMax(id, MotorType.kBrushless), config, "IntakeMotor " +  + id + "  ");
        motor.setInverted(false);
        return motor;
    }

    public static CANCoder createPivotCANCoder(CanDeviceId id){
        CANCoderConfig config = new CANCoderConfig();
        config.sensorDataPeriod = 10;
        config.vBatAndFaultsPeriod = maxStatusFramePeriod;
        config.sensorDirection = true;
		CANCoder encoder = DeviceConfigurationUtil.configCANCoder(new CANCoder(id.getDeviceNumber(), id.getBus()), config, "PivotCANCoder " + id.getDeviceNumber() + "  ");
        return encoder;
    }

    public static TalonSRX createShooterMotor(int id){
        TalonSRXConfig config = new TalonSRXConfig();
        config.enableVoltageCompensation = true;
        TalonSRX motor = DeviceConfigurationUtil.configTalonSRX(new TalonSRX(id), config, "Shooter" + id + "   ");
        motor.setInverted(true);
        motor.setNeutralMode(NeutralMode.Brake);
        return motor;
    }
}