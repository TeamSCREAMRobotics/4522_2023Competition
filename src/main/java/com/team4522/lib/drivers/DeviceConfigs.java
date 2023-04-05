package com.team4522.lib.drivers;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.revrobotics.CANSparkMax.IdleMode;

public class DeviceConfigs {
	public static final int maxStatusFramePeriod = 255;
    public static final double kNominalVoltage = 12.6;

    public static class CANCoderConfig {//TODO make all of the config classes into records when we update to java 17 for the 2024 frc season.
        public int sensorDataPeriod = 20;
        public int vBatAndFaultsPeriod = 20;
        public AbsoluteSensorRange absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;
        public boolean sensorDirection = false;
    }
    
    public static class TalonFXConfig {

        public int status_1_General = 20;
        public int status_2_Feedback0 = 50;
        public int status_3_Quadrature = 50;
        public int status_4_AinTempVbat = 50;
        public int status_6_Misc = maxStatusFramePeriod;
        public int status_7_CommStatus = 20;
        public int status_8_PulseWidth = maxStatusFramePeriod;
        public int status_9_MotProfBuffer = maxStatusFramePeriod;
        public int status_10_Targets = maxStatusFramePeriod;
        public int status_11_UartGadgeteer = maxStatusFramePeriod;
        public int status_12_Feedback1 = maxStatusFramePeriod;
        public int status_13_Base_PIDF0 = 50;
        public int status_14_Turn_PIDF1 = maxStatusFramePeriod;
        public int status_15_FirmwareApiStatus = maxStatusFramePeriod;
        public int status_17_Targets1 = maxStatusFramePeriod;
        public int status_21_FeedbackIntegrated = 50;
        public int status_Brushless_Current = 50;

        public boolean enableVoltageCompensation = true;
        public FeedbackDevice feedbackDevice = FeedbackDevice.IntegratedSensor;
        public StatorCurrentLimitConfiguration statorCurrentLimitConfiguration = new StatorCurrentLimitConfiguration();
        public double neutralDeadband = 0;
        public double voltageCompSaturation = kNominalVoltage;
        public double forwardSoftLimit = Double.MAX_VALUE;
        public double reverseSoftLimit = Double.MIN_VALUE;
        
    }

    public static class TalonSRXConfig{
        public int status_1_General = 50;
		public int status_2_Feedback0 = maxStatusFramePeriod;
		public int status_3_Quadrature = maxStatusFramePeriod;
		public int status_4_AinTempVbat = 50;
		public int status_6_Misc = maxStatusFramePeriod;
		public int status_7_CommStatus = 20;
		public int status_8_PulseWidth = maxStatusFramePeriod;
		public int status_9_MotProfBuffer = maxStatusFramePeriod;
		public int status_10_Targets = maxStatusFramePeriod;
		public int status_11_UartGadgeteer = maxStatusFramePeriod;
		public int status_12_Feedback1 = maxStatusFramePeriod;
		public int status_13_Base_PIDF0 = maxStatusFramePeriod;
		public int status_14_Turn_PIDF1 = maxStatusFramePeriod;
		public int status_15_FirmwareApiStatus = maxStatusFramePeriod;
		public int status_17_Targets1 = maxStatusFramePeriod;
		public int status_21_FeedbackIntegrated = maxStatusFramePeriod;
		public int status_Brushless_Current = 20;
        
        public boolean enableVoltageCompensation = true;
        public FeedbackDevice feedbackDevice = FeedbackDevice.QuadEncoder;	
        public double neutralDeadband = 0;
        public double voltageCompSaturation = kNominalVoltage;
        public double forwardSoftLimit = Double.MAX_VALUE;
        public double reverseSoftLimit = Double.MIN_VALUE;
    }

    public static class Pigeon2Config{
        public int biasedStatus_2_Gyro = maxStatusFramePeriod;
        public int biasedStatus_4_Mag = maxStatusFramePeriod;
        public int biasedStatus_6_Accel = maxStatusFramePeriod;
        public int condStatus_10_SixDeg_Quat = maxStatusFramePeriod;
        public int condStatus_11_GyroAccum = maxStatusFramePeriod;
        public int condStatus_1_General = 20;
        public int condStatus_2_GeneralCompass = maxStatusFramePeriod;
        public int condStatus_3_GeneralAccel = maxStatusFramePeriod;
        public int condStatus_6_SensorFusion = maxStatusFramePeriod;
        public int condStatus_9_SixDeg_YPR = 20;
        public int rawStatus_4_Mag = maxStatusFramePeriod;
    }

    /*        DOCUMENTATION FOR CANSPARKMAX(because their API names are vague)   https://docs.revrobotics.com/sparkmax/operating-modes/control-interfaces
    Periodic Status 0 - Default Rate: 10ms	

    Available Data	Description
    Applied **** Output	The actual value sent to the motors from the motor controller. The frame stores this value as a 16-bit signed integer, and is converted to a floating point value between -1 and 1 by the roboRIO SDK. This value is also used by any follower controllers to set their output.
    Faults	Each bit represents a different fault on the controller. These fault bits clear automatically when the fault goes away.
    Sticky Faults	The same as the Faults field, however the bits do not reset until a power cycle or a 'Clear Faults' command is sent.
    Is Follower	A single bit that is true if the controller is configured to follow another controller.


    Periodic Status 1 - Default Rate: 20ms	

    Available Data	Description
    Motor Velocity	32-bit IEEE floating-point representation of the unconfiguredCANSparkMax velocity in RPM using the selected sensor.
    Motor Temperature	"8-bit unsigned value representing:

    Firmware version 1.0.381 - Voltage of the temperature sensor with 0 = 0V and 255 = 3.3V. Current firmware versions - Motor temperature in °C for the NEO Brushless Motor."
    Motor Voltage	12-bit fixed-point value that is converted to a floating point voltage value (in Volts) by the roboRIO SDK. This is the input voltage to the controller.
    Motor Current	12-bit fixed-point value that is converted to a floating point current value (in Amps) by the roboRIO SDK. This is the raw phase current of the unconfiguredCANSparkMax.


    Periodic Status 2 - Default Rate: 20ms	

    Available Data	Description
    Motor Position	32-bit IEEE floating-point representation of the unconfiguredCANSparkMax position in rotations.


    Periodic Status 3 - Default Rate: 50ms

    Available Data	Description
    Analog Sensor Voltage	10-bit fixed-point value that is converted to a floating point voltage value (in Volts) by the roboRIO SDK. This is the voltage being output by the analog sensor.
    Analog Sensor Velocity	22-bit fixed-point value that is converted to a floating point voltage value (in RPM) by the roboRIO SDK. This is the velocity reported by the analog sensor.
    Analog Sensor Position	32-bit IEEE floating-point representation of the velocity in RPM reported by the analog sensor.


    Periodic Status 4 - Default Rate: 20ms	

    Available Data	Description
    Alternate Encoder Velocity	32-bit IEEE floating-point representation of the velocity in RPM of the alternate encoder.
    Alternate Encoder Position	32-bit IEEE floating-point representation of the position in rotations of the alternate encoder.


    Periodic Status 5 - Default Rate: 200ms	

    Available Data	Description
    Duty Cycle Absolute Encoder Position	32-bit IEEE floating-point representation of the position of the duty cycle absolute encoder.
    Duty Cycle Absolute Encoder Absolute Angle	16-bit integer representation of the absolute angle of the duty cycle absolute encoder.


    Periodic Status 6 - Default Rate: 200ms	

    Available Data	Description
    Duty Cycle Absolute Encoder Velocity	32-bit IEEE floating-point representation of the velocity in RPM of the duty cycle absolute encoder.
    Duty Cycle Absolute Encoder Frequency	16-bit unsigned integer representation of the frequency at which the duty cycle signal is being sent.
     */
    public static class CANSparkMaxConfig{
        public int status0 = 10;
        public int status1 = 50;
        public int status2 = 20;
        public int status3 = 50;
        public int status4 = maxStatusFramePeriod;
        public int status5 = maxStatusFramePeriod;
        public int status6 = maxStatusFramePeriod;

        public boolean enableVoltageCompensation = true;
        public double voltageCompSaturation = kNominalVoltage;
        public IdleMode idleMode = IdleMode.kCoast;
    }
}
