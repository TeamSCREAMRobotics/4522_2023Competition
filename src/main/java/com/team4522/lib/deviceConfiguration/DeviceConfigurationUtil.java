package com.team4522.lib.deviceConfiguration;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
import com.ctre.phoenix.sensors.*;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.team4522.lib.deviceConfiguration.DeviceConfigs.*;
import com.team4522.lib.deviceConfiguration.ErrorChecker.*;

/**This class is responsible for configuring devices with all of their settings, and if there are any errors in configuration, it will repeatedly configure the devices. If this fails,
 * an error will be printed to the console telling us that there is something wrong with the device.
*/
public class DeviceConfigurationUtil {//TODO add in settings as needed. We don't have all of the config variables setup because we don't use most of the motor settings
    public static CANCoder configCANCoder(CANCoder unconfiguredEncoder, CANCoderConfig config, String name){

		DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings() {
                return ErrorChecker.hasConfiguredWithoutErrors(
                    unconfiguredEncoder.configFactoryDefault(),
					unconfiguredEncoder.clearStickyFaults(),
					unconfiguredEncoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, config.sensorDataPeriod),
					unconfiguredEncoder.setStatusFramePeriod(CANCoderStatusFrame.VbatAndFaults, config.vBatAndFaultsPeriod),
					unconfiguredEncoder.configAbsoluteSensorRange(config.absoluteSensorRange),
					unconfiguredEncoder.configSensorDirection(config.sensorDirection)
				);
            }
        };
        ErrorChecker.configureDevice(deviceConfig, name, true);
		
		return unconfiguredEncoder;
	}

	public static TalonFX configTalonFX(TalonFX unconfiguredTalonFX, TalonFXConfig config, String name){

		DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings() {
                return ErrorChecker.hasConfiguredWithoutErrors(
					unconfiguredTalonFX.configFactoryDefault(),
					unconfiguredTalonFX.clearStickyFaults(),

					unconfiguredTalonFX.setStatusFramePeriod(StatusFrame.Status_1_General, config.status_1_General),
					unconfiguredTalonFX.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, config.status_2_Feedback0),
					unconfiguredTalonFX.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, config.status_3_Quadrature),
					unconfiguredTalonFX.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, config.status_4_AinTempVbat),
					unconfiguredTalonFX.setStatusFramePeriod(StatusFrame.Status_6_Misc, config.status_6_Misc),
					unconfiguredTalonFX.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, config.status_7_CommStatus),
					unconfiguredTalonFX.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, config.status_8_PulseWidth),
					unconfiguredTalonFX.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, config.status_9_MotProfBuffer),
					unconfiguredTalonFX.setStatusFramePeriod(StatusFrame.Status_10_Targets, config.status_10_Targets),
					unconfiguredTalonFX.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, config.status_11_UartGadgeteer),
					unconfiguredTalonFX.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, config.status_12_Feedback1),
					unconfiguredTalonFX.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, config.status_13_Base_PIDF0),
					unconfiguredTalonFX.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, config.status_14_Turn_PIDF1),
					unconfiguredTalonFX.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, config.status_15_FirmwareApiStatus),
					unconfiguredTalonFX.setStatusFramePeriod(StatusFrame.Status_17_Targets1, config.status_17_Targets1),
					unconfiguredTalonFX.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, config.status_21_FeedbackIntegrated),
					unconfiguredTalonFX.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, config.status_Brushless_Current),
					unconfiguredTalonFX.configSelectedFeedbackSensor(config.feedbackDevice),
					unconfiguredTalonFX.configStatorCurrentLimit(config.statorCurrentLimitConfiguration),
					unconfiguredTalonFX.configNeutralDeadband(config.neutralDeadband),
					unconfiguredTalonFX.configVoltageCompSaturation(config.voltageCompSaturation),
					unconfiguredTalonFX.configReverseSoftLimitThreshold(config.reverseSoftLimit),
					unconfiguredTalonFX.configForwardSoftLimitThreshold(config.forwardSoftLimit)
                );
            }
        };
		if(config.enableVoltageCompensation) unconfiguredTalonFX.enableVoltageCompensation(true);
        ErrorChecker.configureDevice(deviceConfig, name, true);
		
		return unconfiguredTalonFX;
	}

    
	public static TalonSRX configTalonSRX(TalonSRX unconfiguredTalonSRX, TalonSRXConfig config, String name){

		DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings() {
                return ErrorChecker.hasConfiguredWithoutErrors(
					unconfiguredTalonSRX.configFactoryDefault(),
					unconfiguredTalonSRX.clearStickyFaults(),
					
					unconfiguredTalonSRX.setStatusFramePeriod(StatusFrame.Status_1_General, config.status_1_General),
					unconfiguredTalonSRX.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, config.status_2_Feedback0),
					unconfiguredTalonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, config.status_3_Quadrature),
					unconfiguredTalonSRX.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, config.status_4_AinTempVbat),
					unconfiguredTalonSRX.setStatusFramePeriod(StatusFrame.Status_6_Misc, config.status_6_Misc),
					unconfiguredTalonSRX.setStatusFramePeriod(StatusFrame.Status_7_CommStatus, config.status_7_CommStatus),
					unconfiguredTalonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, config.status_8_PulseWidth),
					unconfiguredTalonSRX.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, config.status_9_MotProfBuffer),
					unconfiguredTalonSRX.setStatusFramePeriod(StatusFrame.Status_10_Targets, config.status_10_Targets),
					unconfiguredTalonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_11_UartGadgeteer, config.status_11_UartGadgeteer),
					unconfiguredTalonSRX.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, config.status_12_Feedback1),
					unconfiguredTalonSRX.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, config.status_13_Base_PIDF0),
					unconfiguredTalonSRX.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, config.status_14_Turn_PIDF1),
					unconfiguredTalonSRX.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, config.status_15_FirmwareApiStatus),
					unconfiguredTalonSRX.setStatusFramePeriod(StatusFrame.Status_17_Targets1, config.status_17_Targets1),
					unconfiguredTalonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_21_FeedbackIntegrated, config.status_21_FeedbackIntegrated),
					unconfiguredTalonSRX.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, config.status_Brushless_Current),
					unconfiguredTalonSRX.configSelectedFeedbackSensor(config.feedbackDevice),
					unconfiguredTalonSRX.configNeutralDeadband(config.neutralDeadband),
					unconfiguredTalonSRX.configVoltageCompSaturation(config.voltageCompSaturation),
					unconfiguredTalonSRX.configForwardSoftLimitThreshold(config.forwardSoftLimit),
					unconfiguredTalonSRX.configReverseSoftLimitThreshold(config.reverseSoftLimit)
                );
            }
        };	
		if(config.enableVoltageCompensation) unconfiguredTalonSRX.enableVoltageCompensation(true);
        ErrorChecker.configureDevice(deviceConfig, name, true);
		
		return unconfiguredTalonSRX;
	}

    public static Pigeon2 configPigeon2(Pigeon2 unconfiguredPigeon2, Pigeon2Config config, String name){
		DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings() {
                return ErrorChecker.hasConfiguredWithoutErrors(
					unconfiguredPigeon2.configFactoryDefault(),
					unconfiguredPigeon2.clearStickyFaults(),
			
					unconfiguredPigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, config.biasedStatus_2_Gyro),
					unconfiguredPigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_4_Mag, config.biasedStatus_4_Mag),
					unconfiguredPigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, config.biasedStatus_6_Accel),
					unconfiguredPigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, config.condStatus_10_SixDeg_Quat),
					unconfiguredPigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_11_GyroAccum, config.condStatus_11_GyroAccum),
					unconfiguredPigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, config.condStatus_1_General),
					unconfiguredPigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_2_GeneralCompass, config.condStatus_2_GeneralCompass),
					unconfiguredPigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, config.condStatus_3_GeneralAccel),
					unconfiguredPigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, config.condStatus_6_SensorFusion),
					unconfiguredPigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, config.condStatus_9_SixDeg_YPR),
					unconfiguredPigeon2.setStatusFramePeriod(PigeonIMU_StatusFrame.RawStatus_4_Mag, config.rawStatus_4_Mag)
                );
            }
        };
        ErrorChecker.configureDevice(deviceConfig, name, true);
		
		return unconfiguredPigeon2;
	}

	
    public static CANSparkMax configCANSparkMax(CANSparkMax unconfiguredCANSparkMax, CANSparkMaxConfig config, String name){

		DeviceConfiguration deviceConfig = new DeviceConfiguration() {
            @Override
            public boolean configureSettings() {
                return ErrorChecker.hasConfiguredWithoutErrors(
					unconfiguredCANSparkMax.restoreFactoryDefaults(),
					unconfiguredCANSparkMax.clearFaults(),
				
					unconfiguredCANSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, config.status0),
					unconfiguredCANSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, config.status0),
					unconfiguredCANSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, config.status2),
					unconfiguredCANSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, config.status3),
					unconfiguredCANSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, config.status4),
					unconfiguredCANSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, config.status5),
					unconfiguredCANSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, config.status6),

					unconfiguredCANSparkMax.setIdleMode(config.idleMode),

					config.enableVoltageCompensation? unconfiguredCANSparkMax.enableVoltageCompensation(config.voltageCompSaturation)
										: unconfiguredCANSparkMax.disableVoltageCompensation()
                );
            }
        };
        ErrorChecker.configureDevice(deviceConfig, name, true);
		
		return unconfiguredCANSparkMax;
	}
}