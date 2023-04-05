package frc2023.auto.actions.lib;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class RaceAction extends ActionBase {
    private List<ActionBase> mActions;
    private boolean mFinished;
    
    private final String mID;

    public RaceAction(List<ActionBase> actions){
        mActions = actions;
        String id = "Race Action: ";
        for(ActionBase a : mActions) id += a.toString();
        mID = id;
    }

    public RaceAction(ActionBase ... actions){
        this(Arrays.asList(actions));
    }

    @Override
    public void run() {
        if(mFinished) return;

        for(ActionBase a : mActions) a.run();
            
        List<ActionBase> newActions = new ArrayList<ActionBase>();

        for(ActionBase a : mActions){
            if(a.isFinished()){
                a.stop(false);
                mFinished = true;
            } else{
                newActions.add(a);
            }
        }

        mActions = newActions;
    }

    @Override
    public void start() {
        
        System.out.println("race action init");
        mFinished = mActions.isEmpty();
        for(ActionBase a : mActions) a.start();
    }

    @Override
    public void stop(boolean interrupted) {
        // if(mActions.isEmpty()) return;
        System.out.println( " Race stop");
        for(ActionBase a : mActions) a.stop(interrupted);
    }

    @Override
    public boolean isFinished() {
        if(mFinished) System.out.println( "RACE ACTION FINISH");
        return mFinished;
    }

    @Override
    public String getID() {
        return mID;
    }
}