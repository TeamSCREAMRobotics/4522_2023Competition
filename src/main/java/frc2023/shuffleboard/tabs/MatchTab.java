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
import frc2023.subsystems.BackLimelight;
import frc2023.subsystems.FrontLimelight;

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
    private final FrontLimelight mFrontLimelight = FrontLimelight.getInstance();
    private final BackLimelight mBackLimelight = BackLimelight.getInstance();

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


    public static ActionBase start1_DoNothing_BLUE = AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE1), Alliance.Blue);
    public static ActionBase start1_DoNothing_RED = AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE1), Alliance.Red);

    
    public static ActionBase start2_DoNothing_BLUE = AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE2), Alliance.Blue);
    public static ActionBase start2_DoNothing_RED = AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE2), Alliance.Red);


    
    public static ActionBase start3_DoNothing_BLUE = AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE3), Alliance.Blue);
    public static ActionBase start3_DoNothing_RED = AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE3), Alliance.Red);
    
    public static ActionBase start4_DoNothing_BLUE = AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE4), Alliance.Blue);
    public static ActionBase start4_DoNothing_RED = AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE4), Alliance.Red);
    
    public static ActionBase start5_DoNothing_BLUE = AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE5), Alliance.Blue);
    public static ActionBase start5_DoNothing_RED = AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE5), Alliance.Red);
    
    public static ActionBase start6_DoNothing_BLUE = AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE6), Alliance.Blue);
    public static ActionBase start6_DoNothing_RED = AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE6), Alliance.Red);
    
    public static ActionBase start7_DoNothing_BLUE = AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE7), Alliance.Blue);
    public static ActionBase start7_DoNothing_RED = AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE7), Alliance.Red);
    
    public static ActionBase start8_DoNothing_BLUE = AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE8), Alliance.Blue);
    public static ActionBase start8_DoNothing_RED = AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE8), Alliance.Red);
    
    public static ActionBase start9_DoNothing_BLUE = AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE9), Alliance.Blue);
    public static ActionBase start9_DoNothing_RED = AutoRoutines.startX_DoNothing(PlacementStates.getSwervePlacementPose(Node.NODE9), Alliance.Red);

    public static ActionBase start1_ScoreCone_BLUE = AutoRoutines.startX_ScoreCone(PlacementStates.getSwervePlacementPose(Node.NODE1), Alliance.Blue);
    public static ActionBase start1_ScoreCone_RED = AutoRoutines.startX_ScoreCone(PlacementStates.getSwervePlacementPose(Node.NODE1), Alliance.Red);
    
    public static ActionBase start3_ScoreCone_BLUE = AutoRoutines.startX_ScoreCone(PlacementStates.getSwervePlacementPose(Node.NODE3), Alliance.Blue);
    public static ActionBase start3_ScoreCone_RED = AutoRoutines.startX_ScoreCone(PlacementStates.getSwervePlacementPose(Node.NODE3), Alliance.Red);
    
    public static ActionBase start4_ScoreCone_BLUE = AutoRoutines.startX_ScoreCone(PlacementStates.getSwervePlacementPose(Node.NODE4), Alliance.Blue);
    public static ActionBase start4_ScoreCone_RED = AutoRoutines.startX_ScoreCone(PlacementStates.getSwervePlacementPose(Node.NODE4), Alliance.Red);
    
    public static ActionBase start6_ScoreCone_BLUE = AutoRoutines.startX_ScoreCone(PlacementStates.getSwervePlacementPose(Node.NODE6), Alliance.Blue);
    public static ActionBase start6_ScoreCone_RED = AutoRoutines.startX_ScoreCone(PlacementStates.getSwervePlacementPose(Node.NODE6), Alliance.Red);
    
    public static ActionBase start7_ScoreCone_BLUE = AutoRoutines.startX_ScoreCone(PlacementStates.getSwervePlacementPose(Node.NODE7), Alliance.Blue);
    public static ActionBase start7_ScoreCone_RED = AutoRoutines.startX_ScoreCone(PlacementStates.getSwervePlacementPose(Node.NODE7), Alliance.Red);
    
    public static ActionBase start9_ScoreCone_BLUE = AutoRoutines.startX_ScoreCone(PlacementStates.getSwervePlacementPose(Node.NODE9), Alliance.Blue);
    public static ActionBase start9_ScoreCone_RED = AutoRoutines.startX_ScoreCone(PlacementStates.getSwervePlacementPose(Node.NODE9), Alliance.Red);

    public static ActionBase start2_ScoreCube_BLUE = AutoRoutines.startX_ScoreCube(PlacementStates.getSwervePlacementPose(Node.NODE2), Alliance.Blue);
    public static ActionBase start2_ScoreCube_RED = AutoRoutines.startX_ScoreCube(PlacementStates.getSwervePlacementPose(Node.NODE2), Alliance.Red);

    
    public static ActionBase start5_ScoreCube_BLUE = AutoRoutines.startX_ScoreCube(PlacementStates.getSwervePlacementPose(Node.NODE5), Alliance.Blue);
    public static ActionBase start5_ScoreCube_RED = AutoRoutines.startX_ScoreCube(PlacementStates.getSwervePlacementPose(Node.NODE5), Alliance.Red);
    
    public static ActionBase start8_ScoreCube_BLUE = AutoRoutines.startX_ScoreCube(PlacementStates.getSwervePlacementPose(Node.NODE8), Alliance.Blue);
    public static ActionBase start8_ScoreCube_RED = AutoRoutines.startX_ScoreCube(PlacementStates.getSwervePlacementPose(Node.NODE8), Alliance.Red);

    public static ActionBase start1_1Cone_2Cube_BLUE = AutoRoutines.start1_1Cone_2Cube(Alliance.Blue);
    public static ActionBase start1_1Cone_2Cube_RED = AutoRoutines.start1_1Cone_2Cube(Alliance.Red);

    public static ActionBase start1_1Cone_1Cube_AutoBalance_BLUE =  AutoRoutines.start1_1Cone_1Cube_AutoBalance(Alliance.Blue);
    public static ActionBase start1_1Cone_1Cube_AutoBalance_RED =  AutoRoutines.start1_1Cone_1Cube_AutoBalance(Alliance.Red);

    public static ActionBase start1_1Cone_2Cube_Coast_BLUE =  AutoRoutines.start1_1Cone_2Cube_Coast(Alliance.Blue);
    public static ActionBase start1_1Cone_2Cube_Coast_RED =  AutoRoutines.start1_1Cone_2Cube_Coast(Alliance.Red);


    public static ActionBase start1_1Cone_1Cube_1PoopShoot_AutoBalance_BLUE =  AutoRoutines.start1_1Cone_1Cube_1PoopShoot_AutoBalance(Alliance.Blue);
    public static ActionBase start1_1Cone_1Cube_1PoopShoot_AutoBalance_RED =  AutoRoutines.start1_1Cone_1Cube_1PoopShoot_AutoBalance(Alliance.Red);

    public static ActionBase start1_1Cone_1Cube_2Poopshoot_Coast_BLUE =  AutoRoutines.start1_1Cone_1Cube_2Poopshoot_Coast(Alliance.Blue);
    public static ActionBase start1_1Cone_1Cube_2Poopshoot_Coast_RED =  AutoRoutines.start1_1Cone_1Cube_2Poopshoot_Coast(Alliance.Red);

    public static ActionBase start1_1Cone_1Cube_2Poopshoot_AutoBalance_BLUE =  AutoRoutines.start1_1Cone_1Cube_2Poopshoot_AutoBalance(Alliance.Blue);
    public static ActionBase start1_1Cone_1Cube_2Poopshoot_AutoBalance_RED =  AutoRoutines.start1_1Cone_1Cube_2Poopshoot_AutoBalance(Alliance.Red);

    public static ActionBase start1_1Cone_3PoopShoot_BLUE = AutoRoutines.start1_1Cone_3PoopShoot(Alliance.Blue);
    public static ActionBase start1_1Cone_3PoopShoot_RED = AutoRoutines.start1_1Cone_3PoopShoot(Alliance.Red);

    public static ActionBase start1_1Cone_3Poopshoot_AutoBalance_BLUE = AutoRoutines.start1_1Cone_3Poopshoot_AutoBalance(Alliance.Blue);
    public static ActionBase start1_1Cone_3Poopshoot_AutoBalance_RED = AutoRoutines.start1_1Cone_3Poopshoot_AutoBalance(Alliance.Red);

    public static ActionBase start1_1Cone_3PoopShoot_Coast_BLUE = AutoRoutines.start1_1Cone_3PoopShoot_Coast(Alliance.Blue);
    public static ActionBase start1_1Cone_3PoopShoot_Coast_RED = AutoRoutines.start1_1Cone_3PoopShoot_Coast(Alliance.Red);


    public static ActionBase start4_1Cone_AutoBalance_BLUE = AutoRoutines.start4_1Cone_AutoBalance(Alliance.Blue);
    public static ActionBase start4_1Cone_AutoBalance_RED = AutoRoutines.start4_1Cone_AutoBalance(Alliance.Red);

    public static ActionBase start4_1Cone_AutoBalanceOverAndBack_BLUE = AutoRoutines.start4_1Cone_AutoBalanceOverAndBack(Alliance.Blue);
    public static ActionBase start4_1Cone_AutoBalanceOverAndBack_RED = AutoRoutines.start4_1Cone_AutoBalanceOverAndBack(Alliance.Red);

    public static ActionBase start5_1Cube_AutoBalance_ChargeStationTarget1_BLUE = AutoRoutines.start5_1Cube_AutoBalance(FieldConstants.chargeStationNode1SideTarget, Alliance.Blue);
    public static ActionBase start5_1Cube_AutoBalance_ChargeStationTarget1_RED = AutoRoutines.start5_1Cube_AutoBalance(FieldConstants.chargeStationNode1SideTarget, Alliance.Red);

    public static ActionBase start5_1Cube_AutoBalanceOverAndBack_ChargeStationTarget1_BLUE = AutoRoutines.start5_1Cube_AutoBalanceOverAndBack(FieldConstants.chargeStationNode1SideTarget, Alliance.Blue);
    public static ActionBase start5_1Cube_AutoBalanceOverAndBack_ChargeStationTarget1_RED = AutoRoutines.start5_1Cube_AutoBalanceOverAndBack(FieldConstants.chargeStationNode1SideTarget, Alliance.Red);

    public static ActionBase start5_1Cube_AutoBalance_ChargeStationTarget9_BLUE = AutoRoutines.start5_1Cube_AutoBalance(FieldConstants.chargeStationNode9SideTarget, Alliance.Blue);
    public static ActionBase start5_1Cube_AutoBalance_ChargeStationTarget9_RED = AutoRoutines.start5_1Cube_AutoBalance(FieldConstants.chargeStationNode9SideTarget, Alliance.Red);

    public static ActionBase start5_1Cube_AutoBalanceOverAndBack_ChargeStationTarget9_BLUE = AutoRoutines.start5_1Cube_AutoBalanceOverAndBack(FieldConstants.chargeStationNode9SideTarget, Alliance.Blue);
    public static ActionBase start5_1Cube_AutoBalanceOverAndBack_ChargeStationTarget9_RED = AutoRoutines.start5_1Cube_AutoBalanceOverAndBack(FieldConstants.chargeStationNode9SideTarget, Alliance.Red);

    public static ActionBase start6_1Cone_AutoBalance_BLUE = AutoRoutines.start6_1Cone_AutoBalance(Alliance.Blue);
    public static ActionBase start6_1Cone_AutoBalance_RED = AutoRoutines.start6_1Cone_AutoBalance(Alliance.Red);

    public static ActionBase start6_1Cone_AutoBalanceOverAndBack_BLUE = AutoRoutines.start6_1Cone_AutoBalanceOverAndBack(Alliance.Blue);
    public static ActionBase start6_1Cone_AutoBalanceOverAndBack_RED = AutoRoutines.start6_1Cone_AutoBalanceOverAndBack(Alliance.Red);


    public static ActionBase start9_1Cone_1Cube_BLUE = AutoRoutines.start9_1Cone_1Cube(Alliance.Blue);
    public static ActionBase start9_1Cone_1Cube_RED = AutoRoutines.start9_1Cone_1Cube(Alliance.Red);

    public static ActionBase start9_1Cone_1Cube_AutoBalance_BLUE = AutoRoutines.start9_1Cone_1Cube_AutoBalance(Alliance.Blue);
    public static ActionBase start9_1Cone_1Cube_AutoBalance_RED = AutoRoutines.start9_1Cone_1Cube_AutoBalance(Alliance.Red);


    public static ActionBase start9_1Cone_1Cube_Coast_BLUE = AutoRoutines.start9_1Cone_1Cube_Coast(Alliance.Blue);
    public static ActionBase start9_1Cone_1Cube_Coast_RED = AutoRoutines.start9_1Cone_1Cube_Coast(Alliance.Red);

    public static ActionBase start9_1Cone_2Cube_BLUE = AutoRoutines.start9_1Cone_2Cube(Alliance.Blue);
    public static ActionBase start9_1Cone_2Cube_RED = AutoRoutines.start9_1Cone_2Cube(Alliance.Red);

    public static ActionBase start9_1Cone_2Cube_CoastBack_BLUE = AutoRoutines.start9_1Cone_2Cube_CoastBack(Alliance.Blue);
    public static ActionBase start9_1Cone_2Cube_CoastBack_RED = AutoRoutines.start9_1Cone_2Cube_CoastBack(Alliance.Red);

    public static ActionBase start9_1Cone_1Cube_GrabCubeDontShoot_BLUE = AutoRoutines.start9_1Cone_1Cube_GrabCubeDontShoot(Alliance.Blue);
    public static ActionBase start9_1Cone_1Cube_GrabCubeDontShoot_RED = AutoRoutines.start9_1Cone_1Cube_GrabCubeDontShoot(Alliance.Red);

    public static ActionBase start9_1Cone_1Cube_1PoopShoot_BLUE = AutoRoutines.start9_1Cone_1Cube_1PoopShoot(Alliance.Blue);
    public static ActionBase start9_1Cone_1Cube_1PoopShoot_RED = AutoRoutines.start9_1Cone_1Cube_1PoopShoot(Alliance.Red);

    public static ActionBase start9_1Cone_2PoopShoot_BLUE = AutoRoutines.start9_1Cone_2PoopShoot(Alliance.Blue);
    public static ActionBase start9_1Cone_2PoopShoot_RED = AutoRoutines.start9_1Cone_2PoopShoot(Alliance.Red);

    public static ActionBase start9_1Cone_1Cube_1PoopShoot_AutoBalance_BLUE = AutoRoutines.start9_1Cone_1Cube_1PoopShoot_AutoBalance(Alliance.Blue);
    public static ActionBase start9_1Cone_1Cube_1PoopShoot_AutoBalance_RED = AutoRoutines.start9_1Cone_1Cube_1PoopShoot_AutoBalance(Alliance.Red);

    public static ActionBase start9_1Cone_1Cube_GrabCubeDontShoot_AutoBalance_BLUE =  AutoRoutines.start9_1Cone_1Cube_GrabCubeDontShoot_AutoBalance(Alliance.Blue);
    public static ActionBase start9_1Cone_1Cube_GrabCubeDontShoot_AutoBalance_RED = AutoRoutines.start9_1Cone_1Cube_GrabCubeDontShoot_AutoBalance(Alliance.Red);

    // public static ActionBase start9_1Cone_1Cube_GrabCubeDontShoot_AutoBalance_BLUE = AutoRoutines.start9_1Cone_1Cube_GrabCubeDontShoot_AutoBalance(Alliance.Blue);
    // public static ActionBase start9_1Cone_1Cube_GrabCubeDontShoot_AutoBalance_RED = AutoRoutines.start9_1Cone_1Cube_GrabCubeDontShoot_AutoBalance(Alliance.Red);

    public static ActionBase start9_1Cone_1Cube_1PoopShoot_Coast_BLUE = AutoRoutines.start9_1Cone_1Cube_1PoopShoot_Coast(Alliance.Blue);
    public static ActionBase start9_1Cone_1Cube_1PoopShoot_Coast_RED = AutoRoutines.start9_1Cone_1Cube_1PoopShoot_Coast(Alliance.Red);

    public static ActionBase start9_1Cone_2PoopShoot_AutoBalance_BLUE = AutoRoutines.start9_1Cone_2PoopShoot_AutoBalance(Alliance.Blue);
    public static ActionBase start9_1Cone_2PoopShoot_AutoBalance_RED = AutoRoutines.start9_1Cone_2PoopShoot_AutoBalance(Alliance.Red);

    public static ActionBase start9_1Cone_1Cube_2PoopShoot_Coast_BLUE = AutoRoutines.start9_1Cone_1Cube_2PoopShoot_Coast(Alliance.Blue);
    public static ActionBase start9_1Cone_1Cube_2PoopShoot_Coast_RED = AutoRoutines.start9_1Cone_1Cube_2PoopShoot_Coast(Alliance.Red);

    public static ActionBase start9_1Cone_3Poopshoot_BLUE = AutoRoutines.start9_1Cone_3Poopshoot(Alliance.Blue);
    public static ActionBase start9_1Cone_3Poopshoot_RED = AutoRoutines.start9_1Cone_3Poopshoot(Alliance.Red);

    public static ActionBase start9_1Cone_3Poopshoot_LeaveCommunity_BLUE = AutoRoutines.start9_1Cone_3Poopshoot_LeaveCommunity(Alliance.Blue);
    public static ActionBase start9_1Cone_3Poopshoot_LeaveCommunity_RED = AutoRoutines.start9_1Cone_3Poopshoot_LeaveCommunity(Alliance.Red);

    public static ActionBase start9_1Cone_3PoopShoot_AutoBalance_BLUE = AutoRoutines.start9_1Cone_3PoopShoot_AutoBalance(Alliance.Blue);
    public static ActionBase start9_1Cone_3PoopShoot_AutoBalance_RED = AutoRoutines.start9_1Cone_3PoopShoot_AutoBalance(Alliance.Red);

    public ActionBase getSelectedAutoRoutine(){//we do this seperately becuase the we want to create the autoroutine when it is called so that it has the right alliance
        Alliance alliance = DriverStation.getAlliance();

        switch(alliance){
            case Blue:
                switch(mAutoChooser.getSelected()){
                    case TEST:
                        return AutoRoutines.getTestRoutine(alliance);
                    case start1_DoNothing:
                        return start1_DoNothing_BLUE;
                    case start2_DoNothing:
                        return start2_DoNothing_BLUE;
                    case start3_DoNothing:
                        return start3_DoNothing_BLUE;
                    case start4_DoNothing:
                        return start4_DoNothing_BLUE;
                    case start5_DoNothing:
                        return start5_DoNothing_BLUE;
                    case start6_DoNothing:
                        return start6_DoNothing_BLUE;
                    case start7_DoNothing:
                        return start7_DoNothing_BLUE;
                    case start8_DoNothing:
                        return start8_DoNothing_BLUE;
                    case start9_DoNothing:
                        return start9_DoNothing_BLUE;

                    case start1_ScoreCone:
                        return start1_ScoreCone_BLUE;
                    case start2_ScoreCube:
                        return start2_ScoreCube_BLUE;
                    case start3_ScoreCone:
                        return start3_ScoreCone_BLUE;  
                    case start4_ScoreCone:
                        return start4_ScoreCone_BLUE;
                    case start5_ScoreCube:
                        return start5_ScoreCube_BLUE;    
                    case start6_ScoreCone:
                        return start6_ScoreCone_BLUE;
                    case start7_ScoreCone:
                        return start7_ScoreCone_BLUE;
                    case start8_ScoreCube:
                        return start8_ScoreCube_BLUE;
                    case start9_ScoreCone:
                        return start9_ScoreCone_BLUE;

                    case start1_1Cone_2Cube:
                        return start1_1Cone_2Cube_BLUE;
                    case start1_1Cone_1Cube_AutoBalance:
                        return start1_1Cone_1Cube_AutoBalance_BLUE;  
                    case start1_1Cone_2Cube_Coast:
                        return start1_1Cone_2Cube_Coast_BLUE;
                    case start1_1Cone_1Cube_1PoopShoot_AutoBalance:
                        return start1_1Cone_1Cube_1PoopShoot_AutoBalance_BLUE;
                    case start1_1Cone_1Cube_2Poopshoot_Coast:
                        return start1_1Cone_1Cube_2Poopshoot_Coast_BLUE;
                    case start1_1Cone_1Cube_2Poopshoot_AutoBalance:
                        return start1_1Cone_1Cube_2Poopshoot_AutoBalance_BLUE;
                    case start1_1Cone_3PoopShoot:
                        return start1_1Cone_3PoopShoot_BLUE;
                    case start1_1Cone_3PoopShoot_AutoBalance:
                        return start1_1Cone_3Poopshoot_AutoBalance_BLUE;
                    case start1_1Cone_3PoopShoot_Coast:
                        return start1_1Cone_3PoopShoot_Coast_BLUE;

                    case start4_1Cone_AutoBalance:
                        return start4_1Cone_AutoBalance_BLUE;
                    case start4_1Cone_AutoBalanceOverAndBack:
                        return start4_1Cone_AutoBalanceOverAndBack_BLUE;
                    case start5_1Cube_AutoBalace_ChargeStationTarget1:
                        return start5_1Cube_AutoBalance_ChargeStationTarget1_BLUE;
                    case start5_1Cube_AutoBalaceOverAndBack_ChargeStationTarget1:
                        return start5_1Cube_AutoBalanceOverAndBack_ChargeStationTarget1_BLUE;
                    case start5_1Cube_AutoBalace_ChargeStationTarget9:
                        return start5_1Cube_AutoBalance_ChargeStationTarget9_BLUE;
                    case start5_1Cube_AutoBalaceOverAndBack_ChargeStationTarget9:
                        return start5_1Cube_AutoBalanceOverAndBack_ChargeStationTarget9_BLUE;
                    case start6_1Cone_AutoBalance:
                        return start6_1Cone_AutoBalance_BLUE;
                    case start6_1Cone_AutoBalanceOverAndBack:
                        return start6_1Cone_AutoBalanceOverAndBack_BLUE;

                    case start9_1Cone_1Cube:
                        return start9_1Cone_1Cube_BLUE;
                    case start9_1Cone_1Cube_AutoBalance:
                        return start9_1Cone_1Cube_AutoBalance_BLUE;
                    case start9_1Cone_1Cube_Coast:
                        return start9_1Cone_1Cube_Coast_BLUE;

                    case start9_1Cone_2Cube:
                        return start9_1Cone_2Cube_BLUE;
                    case start9_1Cone_2Cube_CoastBack:
                        return start9_1Cone_2Cube_CoastBack_BLUE;

                    case start9_1Cone_1Cube_GrabCubeDontShoot:
                        return start9_1Cone_1Cube_GrabCubeDontShoot_BLUE;
                    case start9_1Cone_1Cube_1PoopShoot:
                        return start9_1Cone_1Cube_1PoopShoot_BLUE;
                    case start9_1Cone_2PoopShoot:
                        return start9_1Cone_2PoopShoot_BLUE;
                    case start9_1Cone_1Cube_1PoopShoot_AutoBalance:
                        return start9_1Cone_1Cube_1PoopShoot_AutoBalance_BLUE;
                    case start9_1Cone_1Cube_GrabCubeDontShoot_AutoBalance:
                        return start9_1Cone_1Cube_GrabCubeDontShoot_AutoBalance_BLUE;
                    case start9_1Cone_1Cube_1PoopShoot_Coast:
                        return start9_1Cone_1Cube_1PoopShoot_Coast_BLUE;
                    case start9_1Cone_2PoopShoot_AutoBalance:
                        return start9_1Cone_2PoopShoot_AutoBalance_BLUE;
                    case start9_1Cone_1Cube_2PoopShoot_Coast:
                        return start9_1Cone_1Cube_2PoopShoot_Coast_BLUE;
                    case start9_1Cone_3Poopshoot:
                        return start9_1Cone_3Poopshoot_BLUE;
                    case start9_1Cone_3Poopshoot_LeaveCommunity:
                        return start9_1Cone_3Poopshoot_LeaveCommunity_BLUE;
                    case start9_1Cone_3PoopShoot_AutoBalance:
                        return start9_1Cone_3PoopShoot_AutoBalance_BLUE;
                    case DO_NOTHING:
                        return new EmptyAction();
                }
            break;



                case Red:
                    switch(mAutoChooser.getSelected()){
                    case TEST:
                        return AutoRoutines.getTestRoutine(alliance);
                    case start1_DoNothing:
                        return start1_DoNothing_RED;
                    case start2_DoNothing:
                        return start2_DoNothing_RED;
                    case start3_DoNothing:
                        return start3_DoNothing_RED;
                    case start4_DoNothing:
                        return start4_DoNothing_RED;
                    case start5_DoNothing:
                        return start5_DoNothing_RED;
                    case start6_DoNothing:
                        return start6_DoNothing_RED;
                    case start7_DoNothing:
                        return start7_DoNothing_RED;
                    case start8_DoNothing:
                        return start8_DoNothing_RED;
                    case start9_DoNothing:
                        return start9_DoNothing_RED;

                    case start1_ScoreCone:
                        return start1_ScoreCone_RED;
                    case start2_ScoreCube:
                        return start2_ScoreCube_RED;
                    case start3_ScoreCone:
                        return start3_ScoreCone_RED;  
                    case start4_ScoreCone:
                        return start4_ScoreCone_RED;
                    case start5_ScoreCube:
                        return start5_ScoreCube_RED;    
                    case start6_ScoreCone:
                        return start6_ScoreCone_RED;
                    case start7_ScoreCone:
                        return start7_ScoreCone_RED;
                    case start8_ScoreCube:
                        return start8_ScoreCube_RED;
                    case start9_ScoreCone:
                        return start9_ScoreCone_RED;

                    case start1_1Cone_2Cube:
                        return start1_1Cone_2Cube_RED;
                    case start1_1Cone_1Cube_AutoBalance:
                        return start1_1Cone_1Cube_AutoBalance_RED;  
                    case start1_1Cone_2Cube_Coast:
                        return start1_1Cone_2Cube_Coast_RED;
                    case start1_1Cone_1Cube_1PoopShoot_AutoBalance:
                        return start1_1Cone_1Cube_1PoopShoot_AutoBalance_RED;
                    case start1_1Cone_1Cube_2Poopshoot_Coast:
                        return start1_1Cone_1Cube_2Poopshoot_Coast_RED;
                    case start1_1Cone_1Cube_2Poopshoot_AutoBalance:
                        return start1_1Cone_1Cube_2Poopshoot_AutoBalance_RED;
                    case start1_1Cone_3PoopShoot:
                        return start1_1Cone_3PoopShoot_RED;
                    case start1_1Cone_3PoopShoot_AutoBalance:
                        return start1_1Cone_3Poopshoot_AutoBalance_RED;
                    case start1_1Cone_3PoopShoot_Coast:
                        return start1_1Cone_3PoopShoot_Coast_RED;

                    case start4_1Cone_AutoBalance:
                        return start4_1Cone_AutoBalance_RED;
                    case start4_1Cone_AutoBalanceOverAndBack:
                        return start4_1Cone_AutoBalanceOverAndBack_RED;
                    case start5_1Cube_AutoBalace_ChargeStationTarget1:
                        return start5_1Cube_AutoBalance_ChargeStationTarget1_RED;
                    case start5_1Cube_AutoBalaceOverAndBack_ChargeStationTarget1:
                        return start5_1Cube_AutoBalanceOverAndBack_ChargeStationTarget1_RED;
                    case start5_1Cube_AutoBalace_ChargeStationTarget9:
                        return start5_1Cube_AutoBalance_ChargeStationTarget9_RED;
                    case start5_1Cube_AutoBalaceOverAndBack_ChargeStationTarget9:
                        return start5_1Cube_AutoBalanceOverAndBack_ChargeStationTarget9_RED;
                    case start6_1Cone_AutoBalance:
                        return start6_1Cone_AutoBalance_RED;
                    case start6_1Cone_AutoBalanceOverAndBack:
                        return start6_1Cone_AutoBalanceOverAndBack_RED;

                    case start9_1Cone_1Cube:
                        return start9_1Cone_1Cube_RED;
                    case start9_1Cone_1Cube_AutoBalance:
                        return start9_1Cone_1Cube_AutoBalance_RED;
                    case start9_1Cone_1Cube_Coast:
                        return start9_1Cone_1Cube_Coast_RED;

                    case start9_1Cone_2Cube:
                        return start9_1Cone_2Cube_RED;
                    case start9_1Cone_2Cube_CoastBack:
                        return start9_1Cone_2Cube_CoastBack_RED;

                    case start9_1Cone_1Cube_GrabCubeDontShoot:
                        return start9_1Cone_1Cube_GrabCubeDontShoot_RED;
                    case start9_1Cone_1Cube_1PoopShoot:
                        return start9_1Cone_1Cube_1PoopShoot_RED;
                    case start9_1Cone_2PoopShoot:
                        return start9_1Cone_2PoopShoot_RED;
                    case start9_1Cone_1Cube_1PoopShoot_AutoBalance:
                        return start9_1Cone_1Cube_1PoopShoot_AutoBalance_RED;
                    case start9_1Cone_1Cube_GrabCubeDontShoot_AutoBalance:
                        return start9_1Cone_1Cube_GrabCubeDontShoot_AutoBalance_RED;
                    case start9_1Cone_1Cube_1PoopShoot_Coast:
                        return start9_1Cone_1Cube_1PoopShoot_Coast_RED;
                    case start9_1Cone_2PoopShoot_AutoBalance:
                        return start9_1Cone_2PoopShoot_AutoBalance_RED;
                    case start9_1Cone_1Cube_2PoopShoot_Coast:
                        return start9_1Cone_1Cube_2PoopShoot_Coast_RED;
                    case start9_1Cone_3Poopshoot:
                        return start9_1Cone_3Poopshoot_RED;
                    case start9_1Cone_3Poopshoot_LeaveCommunity:
                        return start9_1Cone_3Poopshoot_LeaveCommunity_RED;
                    case start9_1Cone_3PoopShoot_AutoBalance:
                        return start9_1Cone_3PoopShoot_AutoBalance_RED;
                    case DO_NOTHING:
                        return new EmptyAction();
                }
            break;

            }

            DriverStation.reportWarning("No Routine Selected", false);
            return new EmptyAction();

        }
            



    }
