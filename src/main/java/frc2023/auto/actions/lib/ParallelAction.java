package frc2023.auto.actions.lib;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class ParallelAction extends ActionBase {
    private List<ActionBase> mActions;
    private boolean mFinished;

    private final String mID;

    public ParallelAction(List<ActionBase> actions){
        mActions = actions;
        String id = "Parallel Action: ";
        for(ActionBase a : mActions) id += a.toString();
        mID = id;
    }

    public ParallelAction(ActionBase ... actions){
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
            } else{
                newActions.add(a);
            }
        }
        mActions = newActions;

        if(mActions.isEmpty()) mFinished = true;
    }

    @Override
    public void start() {
        System.out.println("parallel action init");
        mFinished = mActions.isEmpty();
        for(ActionBase a : mActions) a.start();
    }

    @Override
    public void stop(boolean interrupted) {
        if(mActions.isEmpty()) return;
        for(ActionBase a : mActions) a.stop(interrupted);
    }

    @Override
    public boolean isFinished() {
        return mFinished;
    }

    @Override
    public String getID() {
        return mID;
    }
}
