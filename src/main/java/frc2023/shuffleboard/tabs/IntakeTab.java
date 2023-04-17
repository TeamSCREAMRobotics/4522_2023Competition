package frc2023.shuffleboard.tabs;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc2023.shuffleboard.ShuffleboardTabBase;
import frc2023.subsystems.Intake;

public class IntakeTab extends ShuffleboardTabBase{

    private static IntakeTab mInstance = null;
	public static IntakeTab getInstance(){
		if(mInstance == null){
			mInstance = new IntakeTab();
		}
		return mInstance;
	}
    
    private IntakeTab(){}

    private final Intake mIntake = Intake.getInstance();

    private GenericPublisher mIntakeState;
    private GenericPublisher mExtended;
    private GenericPublisher mRollerSpeed;
    private GenericPublisher mUpperConveyorSpeed;
    private GenericPublisher mLowerConveyorSpeed;

    
    private GenericPublisher mRodExtended;
    private GenericPublisher mRodRawPosition;
    
    @Override
    public void createEntries() {
        mTab = Shuffleboard.getTab("Intake");
        mIntakeState = createStringEntry("State", mIntake.getLastState().toString());
        mExtended = createBooleanEntry("Setting Extended", false);
        mRollerSpeed = createNumberEntry("Roller Speed", 0.0);
        
        mUpperConveyorSpeed = createNumberEntry("Upper Conveyor Speed", 0.0);
        mLowerConveyorSpeed = createNumberEntry("Lower Conveyor Speed", 0.0);

        mRodExtended = createBooleanEntry("Rod Extened", false);
        mRodRawPosition = createNumberEntry("Rod Raw Position", 0);
    }

    @Override
    public void update() {
        mIntakeState.setString(mIntake.getLastState().toString());
        mExtended.setBoolean(mIntake.isExtended());
        mRollerSpeed.setDouble(mIntake.getRollerSpeed());
        mUpperConveyorSpeed.setDouble(mIntake.getUpperConveyorSpeed());
        mLowerConveyorSpeed.setDouble(mIntake.getLowerConveyorSpeed());
        mRodExtended.setBoolean(mIntake.getRodExtended());
        mRodRawPosition.setDouble(mIntake.getRodRawPosition());
    }
}