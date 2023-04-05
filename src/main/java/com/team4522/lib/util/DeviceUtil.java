package com.team4522.lib.util;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.team4522.lib.drivers.ErrorChecker;
import com.team4522.lib.drivers.ErrorChecker.DeviceConfiguration;
import com.team4522.lib.pid.MotionMagicConstants;
import com.team4522.lib.pid.PIDConstants;

public class DeviceUtil {
    
    public static void configTalonFXPID(TalonFX motor, PIDConstants pidConstants, boolean printInfo){
		DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings() {
                return ErrorChecker.hasConfiguredWithoutErrors(
                    motor.config_kP(0, pidConstants.kP()),
					motor.config_kI(0, pidConstants.kI()),
					motor.config_kD(0, pidConstants.kD()),
					motor.config_kF(0, pidConstants.kF()),
					motor.config_IntegralZone(0, pidConstants.integralZone()),
					motor.configMaxIntegralAccumulator(0, pidConstants.maxIntegralAccumulator()),
					motor.configClosedLoopPeakOutput(0, pidConstants.maxOutput())
                );
            }
        };
        ErrorChecker.configureDevice(deviceConfig, "TalonFXPID", printInfo);
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
    
    public static void configTalonSRXPID(TalonSRX motor, PIDConstants pidConstants, boolean printInfo){
		DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings() {
                return ErrorChecker.hasConfiguredWithoutErrors(
					motor.config_kP(0, pidConstants.kP()),
					motor.config_kI(0, pidConstants.kI()),
					motor.config_kD(0, pidConstants.kD()),
					motor.config_kF(0, pidConstants.kF()),
					motor.config_IntegralZone(0, pidConstants.integralZone()),
					motor.configMaxIntegralAccumulator(0, pidConstants.maxIntegralAccumulator()),
					motor.configClosedLoopPeakOutput(0, pidConstants.maxOutput())
                );
            }
        };
        ErrorChecker.configureDevice(deviceConfig, "TalonSRXPID", printInfo);
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