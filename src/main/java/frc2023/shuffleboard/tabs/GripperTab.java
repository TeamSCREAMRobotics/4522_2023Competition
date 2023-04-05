package frc2023.shuffleboard.tabs;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc2023.shuffleboard.ShuffleboardTabBase;
import frc2023.subsystems.Gripper;

public class GripperTab extends ShuffleboardTabBase{

    private static GripperTab mInstance = null;
	public static GripperTab getInstance(){
		if(mInstance == null){
			mInstance = new GripperTab();
		}
		return mInstance;
	}
    
    private GripperTab(){}

    private final Gripper mGripper = Gripper.getInstance();

    private GenericPublisher mGripperState;
    @Override
    public void createEntries() {
        mTab = Shuffleboard.getTab("Gripper");
        mGripperState = createStringEntry("State", mGripper.getLastState().toString());
    }

    @Override
    public void update() {
        mGripperState.setString(mGripper.getLastState().toString());
    }
}