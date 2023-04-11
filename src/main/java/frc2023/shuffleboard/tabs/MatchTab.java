package frc2023.shuffleboard.tabs;

import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc2023.PlacementStates;
import frc2023.Constants.FieldConstants;
import frc2023.PlacementStates.Node;
import frc2023.auto.AutoRoutines;
import frc2023.auto.actions.lib.ActionBase;
import frc2023.auto.actions.lib.EmptyAction;
import frc2023.auto.modes.AutoRoutineExecutor.AutoRoutine;
import frc2023.battery.BatteryManager;
import frc2023.controlboard.ControlBoard;
import frc2023.shuffleboard.ShuffleboardTabBase;
import frc2023.subsystems.Limelight;

public class MatchTab extends ShuffleboardTabBase {

    private final BatteryManager mBatteryManager = BatteryManager.getInstance();
    private final ControlBoard mControlBoard = ControlBoard.getInstance();

    private GenericPublisher mCurrentDraw;
    private GenericPublisher mVoltage;
    private GenericPublisher mPowerConsumption;

    private GenericPublisher mSelectedNode;
    private GenericPublisher mSelectedLevel;
    private GenericPublisher mSelectedGamePiece;
    private GenericPublisher mArmPositionPlace;
    private GenericPublisher mManualOverride;
    private GenericPublisher mFrontLimelightPipeline;
    private GenericPublisher mBackLimelightPipeline;



    private SendableChooser<AutoRoutine> mAutoChooser = new SendableChooser<>();
    private final Limelight mFrontLimelight = Limelight.getFrontInstance();
    private final Limelight mBackLimelight = Limelight.getBackInstance();

    private static MatchTab mInstance = null;
	public static MatchTab getInstance(){
		if(mInstance == null){
			mInstance = new MatchTab();
		}
		return mInstance;
	}

    @Override
    public void createEntries() {

        mAutoChooser.setDefaultOption(AutoRoutine.DO_NOTHING.toString(), AutoRoutine.DO_NOTHING);
        for(AutoRoutine autoRoutine : AutoRoutine.values()){
            mAutoChooser.addOption(autoRoutine.toString(), autoRoutine);
        }

        mTab = Shuffleboard.getTab("Match");

        mCurrentDraw = createNumberEntry("Current Draw", 0.0);
        mVoltage = createNumberEntry("Voltage", 0.0);
        mPowerConsumption = createNumberEntry("Power Consumption", 0.0);

        mSelectedNode = createStringEntry("Selected Node", "N/A");
        mSelectedLevel = createStringEntry("Selected level", "N/A");
        mSelectedGamePiece = createStringEntry("Selected GamePiece", "N/A");
        mArmPositionPlace = createEntry("Position Place", false);
        mManualOverride = createEntry("Manual Override", false);
        mFrontLimelightPipeline = createNumberEntry("Front Limelight Pipeline", -1);
        mBackLimelightPipeline = createNumberEntry("Back Limelight Pipeline", -1);
        
        mTab.add("Selected Auto", mAutoChooser);
    }

    @Override
    public void update() {
        mCurrentDraw.setDouble(mBatteryManager.getCurrentDraw());
        mVoltage.setDouble(mBatteryManager.getBatteryVoltage());
        mPowerConsumption.setDouble(mBatteryManager.getPowerConsumption());

        mSelectedNode.setString(mControlBoard.getSelectedNode().toString());
        mSelectedLevel.setString(mControlBoard.getSelectedLevel().toString());
        mSelectedGamePiece.setString(mControlBoard.getSelectedNode().getType().toString());
        mArmPositionPlace.setBoolean(mControlBoard.armPlaceCone());
        mManualOverride.setBoolean(mControlBoard.getArmManualOverride());
        mFrontLimelightPipeline.setInteger(mFrontLimelight.getPipeline());
        mBackLimelightPipeline.setInteger(mBackLimelight.getPipeline());
    }

    public ActionBase getSelectedAutoRoutine(){//we do this seperately becuase the we want to create the autoroutine when it is called so that it has the right alliance
        Alliance alliance = DriverStation.getAlliance();
        // System.out.println(mAutoChooser.getSelected());
        switch(mAutoChooser.getSelected()){
            case TEST:
                return AutoRoutines.getTestRoutine(alliance);
            case start1_DoNothing:
                return AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE1), alliance);
            case start2_DoNothing:
                return AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE2), alliance);
            case start3_DoNothing:
                return AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE3), alliance);
            case start4_DoNothing:
                return AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE4), alliance);
            case start5_DoNothing:
                return AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE5), alliance);
            case start6_DoNothing:
                return AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE6), alliance);
            case start7_DoNothing:
                return AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE7), alliance);
            case start8_DoNothing:
                return AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE8), alliance);
            case start9_DoNothing:
                return AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE9), alliance);

            case start1_ScoreCone:
                return AutoRoutines.startX_ScoreCone(PlacementStates.getSwervePlacementPose(Node.NODE1), alliance);
            case start2_ScoreCube:
                return AutoRoutines.startX_ScoreCube(PlacementStates.getSwervePlacementPose(Node.NODE2), alliance);
            case start3_ScoreCone:
                return AutoRoutines.startX_ScoreCone(PlacementStates.getSwervePlacementPose(Node.NODE3), alliance);  
            case start4_ScoreCone:
                return AutoRoutines.startX_ScoreCone(PlacementStates.getSwervePlacementPose(Node.NODE4), alliance);
            case start5_ScoreCube:
                return AutoRoutines.startX_ScoreCube(PlacementStates.getSwervePlacementPose(Node.NODE5), alliance);    
            case start6_ScoreCone:
                return AutoRoutines.startX_ScoreCone(PlacementStates.getSwervePlacementPose(Node.NODE6), alliance);
            case start7_ScoreCone:
                return AutoRoutines.startX_ScoreCone(PlacementStates.getSwervePlacementPose(Node.NODE7), alliance);
            case start8_ScoreCube:
                return AutoRoutines.startX_ScoreCube(PlacementStates.getSwervePlacementPose(Node.NODE8), alliance);
            case start9_ScoreCone:
                return AutoRoutines.startX_ScoreCone(PlacementStates.getSwervePlacementPose(Node.NODE9), alliance);

            case start1_1Cone_2Cube:
                return AutoRoutines.start1_1Cone_2Cube(alliance);
            case start1_1Cone_1Cube_AutoBalance:
                return AutoRoutines.start1_1Cone_1Cube_AutoBalance(alliance);  
            case start1_1Cone_2Cube_Coast:
                return AutoRoutines.start1_1Cone_2Cube_Coast(alliance);
            case start1_1Cone_1Cube_1PoopShoot_AutoBalance:
                return AutoRoutines.start1_1Cone_1Cube_1PoopShoot_AutoBalance(alliance);
            case start1_1Cone_1Cube_2Poopshoot_Coast:
                return AutoRoutines.start1_1Cone_1Cube_2Poopshoot_Coast(alliance);
            case start1_1Cone_1Cube_2Poopshoot_AutoBalance:
                return AutoRoutines.start1_1Cone_1Cube_2Poopshoot_AutoBalance(alliance);
            case start1_1Cone_3PoopShoot:
                return AutoRoutines.start1_1Cone_3PoopShoot(alliance);
            case start1_1Cone_3PoopShoot_AutoBalance:
                return AutoRoutines.start1_1Cone_3Poopshoot_AutoBalance(alliance);
            case start1_1Cone_3PoopShoot_Coast:
                return AutoRoutines.start1_1Cone_3PoopShoot_Coast(alliance);

            case start4_1Cone_AutoBalance:
                return AutoRoutines.start4_1Cone_AutoBalance(alliance);
            case start4_1Cone_AutoBalanceOverAndBack:
                return AutoRoutines.start4_1Cone_AutoBalanceOverAndBack(alliance);
            case start5_1Cube_AutoBalace_ChargeStationTarget1:
                return AutoRoutines.start5_1Cube_AutoBalance(FieldConstants.chargeStationNode1SideTarget, alliance);
            case start5_1Cube_AutoBalaceOverAndBack_ChargeStationTarget1:
                return AutoRoutines.start5_1Cube_AutoBalanceOverAndBack(FieldConstants.chargeStationNode1SideTarget, alliance);
            case start5_1Cube_AutoBalace_ChargeStationTarget9:
                return AutoRoutines.start5_1Cube_AutoBalance(FieldConstants.chargeStationNode9SideTarget, alliance);
            case start5_1Cube_AutoBalaceOverAndBack_ChargeStationTarget9:
                return AutoRoutines.start5_1Cube_AutoBalanceOverAndBack(FieldConstants.chargeStationNode9SideTarget, alliance);
            case start6_1Cone_AutoBalance:
                return AutoRoutines.start6_1Cone_AutoBalance(alliance);
            case start6_1Cone_AutoBalanceOverAndBack:
                return AutoRoutines.start6_1Cone_AutoBalanceOverAndBack(alliance);   

            case start9_1Cone_1Cube:
                return AutoRoutines.start9_1Cone_1Cube(alliance);
            case start9_1Cone_1Cube_AutoBalance:
                return AutoRoutines.start9_1Cone_1Cube_AutoBalance(alliance);
            case start9_1Cone_1Cube_Coast:
                return AutoRoutines.start9_1Cone_1Cube_Coast(alliance);
            case start9_1Cone_1Cube_GrabCubeDontShoot:
                return AutoRoutines.start9_1Cone_1Cube_GrabCubeDontShoot(alliance);
            case start9_1Cone_1Cube_1PoopShoot:
                return AutoRoutines.start9_1Cone_1Cube_1PoopShoot(alliance);
            case start9_1Cone_2PoopShoot:
                return AutoRoutines.start9_1Cone_2PoopShoot(alliance);
            case start9_1Cone_1Cube_1PoopShoot_AutoBalance:
                return AutoRoutines.start9_1Cone_1Cube_1PoopShoot_AutoBalance(alliance);
            case start9_1Cone_1Cube_GrabCubeDontShoot_AutoBalance:
                return AutoRoutines.start9_1Cone_1Cube_GrabCubeDontShoot_AutoBalance(alliance);
            case start9_1Cone_1Cube_1PoopShoot_Coast:
                return AutoRoutines.start9_1Cone_1Cube_1PoopShoot_Coast(alliance);
            case start9_1Cone_2PoopShoot_AutoBalance:
                return AutoRoutines.start9_1Cone_2PoopShoot_AutoBalance(alliance);
            case start9_1Cone_1Cube_2PoopShoot_Coast:
                return AutoRoutines.start9_1Cone_1Cube_2PoopShoot_Coast(alliance);
            case start9_1Cone_3Poopshoot:
                return AutoRoutines.start9_1Cone_3Poopshoot(alliance);
            case start9_1Cone_3Poopshoot_LeaveCommunity:
                return AutoRoutines.start9_1Cone_3Poopshoot_LeaveCommunity(alliance);
            case start9_1Cone_3PoopShoot_AutoBalance:
                return AutoRoutines.start9_1Cone_3PoopShoot_AutoBalance(alliance);
            default:
                DriverStation.reportWarning("No Routine Selected", false);
            case DO_NOTHING:
                return new EmptyAction();
          }
    }

}