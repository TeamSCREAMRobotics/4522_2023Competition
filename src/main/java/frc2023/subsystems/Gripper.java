package frc2023.subsystems;

import edu.wpi.first.wpilibj.Solenoid;

public class Gripper extends Subsystem{

	public final PeriodicIO mPeriodicIO = new PeriodicIO();
	private final Solenoid mUpperSolenoid;
	private final Solenoid mToiletSeatSolenoid;
    private final Devices mDevices = Devices.getInstance();
	
	private Gripper(){
		mUpperSolenoid =  mDevices.dUpperGripperSolenoid;
		mToiletSeatSolenoid = mDevices.dLowerGripperSolenoid;
	}
	
	private static Gripper mInstance = null;
	public static Gripper getInstance(){
		if(mInstance == null){
			mInstance = new Gripper();
		}
		return mInstance;
	}

	public enum GripperState{
		DISABLED, PLACE_CONE, CLOSED, FULL_OPEN;
	}

	public class PeriodicIO{
		//Inputs
		public GripperState lastState = GripperState.DISABLED;
		public boolean beamBrokenLastLoop = false;
		public boolean seesGamePiece = false;
		public boolean hasGamePiece = false;

		//Outputs
		public GripperState state = GripperState.DISABLED;
	}
	
	@Override
	public void disable(){
		mPeriodicIO.state = GripperState.DISABLED;
	}

    public void placeCone(){
		mPeriodicIO.state = GripperState.PLACE_CONE;
	}

    public void openForConeIntake(){
        mPeriodicIO.state = GripperState.PLACE_CONE;//this is not a bug
    }

    public void close(){
		mPeriodicIO.state = GripperState.CLOSED;
	}

	public void fullOpen(){
		mPeriodicIO.state = GripperState.FULL_OPEN;
	}

	public GripperState get(){
		return mPeriodicIO.state;
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
			case PLACE_CONE:
				mUpperSolenoid.set(true);
				mToiletSeatSolenoid.set(false);
				break;
            case CLOSED:
                mUpperSolenoid.set(false);
				mToiletSeatSolenoid.set(false);
                break;
			case FULL_OPEN:
				mUpperSolenoid.set(true);
				mToiletSeatSolenoid.set(true);
				break;
		}
		mPeriodicIO.lastState = mPeriodicIO.state;
	}
	
	@Override
	public void outputTelemetry() {
		
	}
}