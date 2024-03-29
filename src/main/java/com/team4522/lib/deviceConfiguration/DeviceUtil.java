package com.team4522.lib.deviceConfiguration;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.team4522.lib.deviceConfiguration.ErrorChecker.DeviceConfiguration;
import com.team4522.lib.pid.MotionMagicConstants;
import com.team4522.lib.pid.PIDConstants;

public class DeviceUtil {
    
    public static void configTalonFXPID(TalonFX motor, PIDConstants pidConstants, boolean printInfo, int slotID){
		DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings() {
                return ErrorChecker.hasConfiguredWithoutErrors(
                    motor.config_kP(slotID, pidConstants.kP()),
					motor.config_kI(slotID, pidConstants.kI()),
					motor.config_kD(slotID, pidConstants.kD()),
					motor.config_kF(slotID, pidConstants.kF()),
					motor.config_IntegralZone(0, pidConstants.integralZone()),
					motor.configMaxIntegralAccumulator(slotID, pidConstants.maxIntegralAccumulator()),
					motor.configClosedLoopPeakOutput(slotID, pidConstants.maxOutput())
                );
            }
        };
        ErrorChecker.configureDevice(deviceConfig, "TalonFXPID", printInfo);
	}

	    
    public static void configTalonFXPID(TalonFX motor, PIDConstants pidConstants, boolean printInfo){
		configTalonFXPID(motor, pidConstants, printInfo, 0);
	}


	public static void configTalonFXMotionMagic(TalonFX motor, MotionMagicConstants motionMagicConstants, boolean printInfo){
		DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings() {
                return ErrorChecker.hasConfiguredWithoutErrors(
					motor.configMotionCruiseVelocity(motionMagicConstants.cruiseVelocity),
					motor.configMotionAcceleration(motionMagicConstants.acceleration),
					motor.configMotionSCurveStrength(motionMagicConstants.sCurveStrength)
                );
            }
        };
        ErrorChecker.configureDevice(deviceConfig, "TalonFXMotionMagic", printInfo);
	}

	public static void configTalonSRXMotionMagic(TalonSRX motor, MotionMagicConstants motionMagicConstants, boolean printInfo){

		DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings() {
                return ErrorChecker.hasConfiguredWithoutErrors(
					motor.configMotionCruiseVelocity(motionMagicConstants.cruiseVelocity),
					motor.configMotionAcceleration(motionMagicConstants.acceleration),
					motor.configMotionSCurveStrength(motionMagicConstants.sCurveStrength)
                );
            }
        };
        ErrorChecker.configureDevice(deviceConfig, "TalonSRXMotionMagic", printInfo);
	}
    
    public static void configTalonSRXPID(TalonSRX motor, PIDConstants pidConstants, boolean printInfo, int slotID){
		DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings() {
                return ErrorChecker.hasConfiguredWithoutErrors(
					motor.config_kP(slotID, pidConstants.kP()),
					motor.config_kI(slotID, pidConstants.kI()),
					motor.config_kD(slotID, pidConstants.kD()),
					motor.config_kF(slotID, pidConstants.kF()),
					motor.config_IntegralZone(slotID, pidConstants.integralZone()),
					motor.configMaxIntegralAccumulator(slotID, pidConstants.maxIntegralAccumulator()),
					motor.configClosedLoopPeakOutput(slotID, pidConstants.maxOutput())
                );
            }
        };
        ErrorChecker.configureDevice(deviceConfig, "TalonSRXPID", printInfo);
	}


    public static void configTalonSRXPID(TalonSRX motor, PIDConstants pidConstants, boolean printInfo){
		configTalonSRXPID(motor, pidConstants, printInfo, 0);
	}

	public static void configCANSparkMaxPID(CANSparkMax motor, PIDConstants pidConstants, boolean printInfo){
		SparkMaxPIDController pidController = motor.getPIDController();

		DeviceConfiguration config = new DeviceConfiguration() {
			@Override
			public boolean configureSettings() {
				return ErrorChecker.hasConfiguredWithoutErrors(
					pidController.setP(pidConstants.kP()),
					pidController.setI(pidConstants.kI()),
					pidController.setD(pidConstants.kD()),
					pidController.setFF(pidConstants.kF()),
					pidController.setIZone(pidConstants.integralZone()),
					pidController.setIMaxAccum(pidConstants.maxIntegralAccumulator(), 0),
					pidController.setOutputRange(pidConstants.minOutput(), pidConstants.maxOutput())
				);
			}
		};
        ErrorChecker.configureDevice(config, "TalonSRXPID", printInfo);
	}	
}