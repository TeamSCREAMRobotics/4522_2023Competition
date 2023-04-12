package frc2023;

import com.team254.CanDeviceId;


public class Ports {
    public static final String canivoreBusName = "canivore";
	// Controller ports
	public static final int driverControllerPort = 0;
	public static final int buttonboardPortA = 1;
	public static final int buttonboardPortB = 2;

    //Electrical component ports
    public static final int pneumaticsHubID = 1;
    public static final int pdhID = 1;

    //Intake Motor Ports
    public static final int intakeMotorID = 2;
    public static final int upperConveyorID = 3;
    public static final int lowerConveyorID = 1;
    public static final int shooterID = 7;
    public static final int rodID = 8;

    //Arm Motor Ports
    public static final CanDeviceId ArmPivotID = new CanDeviceId(4);
    public static final int ArmTelescopeID = 5;

    //Sensor Ports
    public static final CanDeviceId pigeonID = new CanDeviceId(0, canivoreBusName);
    public static final int beamBreakID = 3; 
    public static final CanDeviceId ArmPivotEncoderID = new CanDeviceId(1);

    //Solenoid Ports
    public static final int intakeSolenoidID = 7;
    public static final int gripperUpperSolenoidID = 15;

    //Swerve Module Ports. 
    // We setup all 8 of our modules with different IDs and offsets so that they can be "Hot swapped". All we have to do is select which
    // modules we are using and which location they are atand we are using in constants. 
    public static final ModuleIDs module1IDs = new ModuleIDs(11, 12, 2, canivoreBusName);
    public static final ModuleIDs module2IDs = new ModuleIDs(13, 14, 3, canivoreBusName);
    public static final ModuleIDs module3IDs = new ModuleIDs(15, 16, 4, canivoreBusName);
    public static final ModuleIDs module4IDs = new ModuleIDs(17, 18, 5, canivoreBusName);
    public static final ModuleIDs module5IDs = new ModuleIDs(19, 20, 6, canivoreBusName);
    public static final ModuleIDs module6IDs = new ModuleIDs(21, 22, 7, canivoreBusName);
    public static final ModuleIDs module7IDs = new ModuleIDs(23, 24, 8, canivoreBusName);
    public static final ModuleIDs module8IDs = new ModuleIDs(25, 26, 9, canivoreBusName);

    //Limelight network table names
    public static final String frontLimelightName = "limelight-front";
    public static final String backLimelightName = "limelight-back";

    public static class ModuleIDs{
        public final CanDeviceId driveID;
        public final CanDeviceId steerID;
        public final CanDeviceId cancoderID;
        public final String busName;
        public ModuleIDs(int driveID, int steerID, int cancoderID, String busName){
            this.driveID = new CanDeviceId(driveID, busName);
            this.steerID = new CanDeviceId(steerID, busName);
            this.cancoderID = new CanDeviceId(cancoderID, busName);
            this.busName = busName;
        }
    }
}