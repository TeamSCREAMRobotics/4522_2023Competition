package frc2023.subsystems;

import edu.wpi.first.wpilibj.Solenoid;

public class Gripper extends Subsystem{

	public final PeriodicIO mPeriodicIO = new PeriodicIO();
	private final Solenoid mUpperSolenoid;
    private final Devices mDevices = Devices.getInstance();
	
	private Gripper(){
		mUpperSolenoid = mDevices.dUpperGripperSolenoid;
	}
	
	private static Gripper mInstance = null;
	public static Gripper getInstance(){
		if(mInstance == null){
			mInstance = new Gripper();
		}
		return mInstance;
	}

	public enum GripperState{
		DISABLED, OPEN, CLOSED;
	}

	public class PeriodicIO{
		//Inputs
		public GripperState lastState = GripperState.DISABLED;

		//Outputs
		public GripperState state = GripperState.DISABLED;
	}
	
	@Override
	public void disable(){
		mPeriodicIO.state = GripperState.DISABLED;
	}

    public void open(){
		mPeriodicIO.state = GripperState.OPEN;
	}

    public void close(){
		mPeriodicIO.state = GripperState.CLOSED;
	}
	
	public GripperState getLastState(){
		return mPeriodicIO.lastState;
	}

	@Override
	public void stop() {
		mPeriodicIO.state = GripperState.DISABLED;
	}

	@Override
	public void writeOutputs() {
		switch(mPeriodicIO.state){
			case DISABLED:
				//literally does nothing while disabled
                break;
			case OPEN:
				mUpperSolenoid.set(true);
				break;
            case CLOSED:
                mUpperSolenoid.set(false);
                break;
		}
		mPeriodicIO.lastState = mPeriodicIO.state;
	}
	
	@Override
	public void outputTelemetry() {
		
	}
}