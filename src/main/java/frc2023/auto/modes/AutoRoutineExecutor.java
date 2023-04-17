package frc2023.auto.modes;


import frc2023.auto.actions.lib.ActionBase;
import frc2023.auto.actions.lib.EmptyAction;

public class AutoRoutineExecutor {

    private ActionBase mSelectedRoutine = new EmptyAction();//We could make this an optional and if no auto is selected then not run an auto, but making it start as the doNothingRoutine is simpler
    private boolean mFinished = false;

    private AutoRoutineExecutor(){}
    
    private static AutoRoutineExecutor mInstance = null;
    public static AutoRoutineExecutor getInstance(){
        if(mInstance == null){
            mInstance = new AutoRoutineExecutor();
        }
        return mInstance;
    }

    public enum AutoRoutine {
        DO_NOTHING, TEST, start1_DoNothing, start2_DoNothing, start3_DoNothing, start4_DoNothing, start5_DoNothing, start6_DoNothing, start7_DoNothing, start8_DoNothing, start9_DoNothing, 
        
        start1_ScoreCone, start2_ScoreCube, start3_ScoreCone, start4_ScoreCone, start5_ScoreCube, start6_ScoreCone, start7_ScoreCone, start8_ScoreCube, start9_ScoreCone,   
        
        
        start4_1Cone_AutoBalance, start4_1Cone_AutoBalanceOverAndBack,
        start5_1Cube_AutoBalace_ChargeStationTarget1, start5_1Cube_AutoBalaceOverAndBack_ChargeStationTarget1, start5_1Cube_AutoBalace_ChargeStationTarget9, start5_1Cube_AutoBalaceOverAndBack_ChargeStationTarget9,
        start6_1Cone_AutoBalance, start6_1Cone_AutoBalanceOverAndBack,

        start1_1Cone_2Cube, start1_1Cone_2Cube_Coast, start1_1Cone_1Cube_2Poopshoot_Coast, start1_1Cone_3PoopShoot, start1_1Cone_3PoopShoot_Coast,
        start1_1Cone_3PoopShoot_AutoBalance, start1_1Cone_1Cube_1PoopShoot_AutoBalance, start1_1Cone_1Cube_2Poopshoot_AutoBalance, start1_1Cone_1Cube_AutoBalance, 
        

        start9_1Cone_1Cube, start9_1Cone_1Cube_Coast, start9_1Cone_3Poopshoot, start9_1Cone_3Poopshoot_LeaveCommunity, start9_1Cone_2Cube, start9_1Cone_2Cube_CoastBack, start9_1Cone_1Cube_1PoopShoot, start9_1Cone_2PoopShoot, start9_1Cone_1Cube_1PoopShoot_Coast, start9_1Cone_1Cube_GrabCubeDontShoot,
        start9_1Cone_2PoopShoot_AutoBalance, start9_1Cone_1Cube_1PoopShoot_AutoBalance, start9_1Cone_1Cube_AutoBalance, start9_1Cone_3PoopShoot_AutoBalance, start9_1Cone_1Cube_2PoopShoot_Coast, start9_1Cone_1Cube_GrabCubeDontShoot_AutoBalance,  
    }

    public void selectRoutine(ActionBase routine){
        
        mSelectedRoutine = routine;
        mFinished = false;
    }


    public void start(){
        System.out.println("autoRoutine start");
        mFinished = false;
        mSelectedRoutine.start();
    }

    public void stop(){
        if(!mFinished){
            mFinished = true;
            mSelectedRoutine.stop(true);
        }
    }

    public void execute(){
        if(mFinished) return;
        mSelectedRoutine.run();
        if(mSelectedRoutine.isFinished() && !mFinished){
            mFinished = true;
            mSelectedRoutine.stop(false);
        }
    }

    public boolean isFinished(){
        return mFinished;
    }
    
    public ActionBase getAutoRoutine(){
        return mSelectedRoutine;
    }
}