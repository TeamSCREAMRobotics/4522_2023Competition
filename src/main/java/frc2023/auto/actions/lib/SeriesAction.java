package frc2023.auto.actions.lib;

import java.util.Arrays;
import java.util.List;

public class SeriesAction extends ActionBase {
    private int mActiveIndex;
    private List<ActionBase> mActions;
    private boolean mFinished;

    private final String mID;

    public SeriesAction(List<ActionBase> actions){
        mActions = actions;
        
        String id = "Series Action: ";
        for(ActionBase a : mActions) id += a.toString();
        mID = id;
    }

    public SeriesAction(ActionBase ... actions){
        this(Arrays.asList(actions));
    }

    @Override
    public void run() {
        if(mFinished) return;

        ActionBase active = getActive();
        active.run();

        if(active.isFinished()){
            if(mActiveIndex == mActions.size()-1){
                mFinished = true;
                return;
            }
            active.stop(false);
            mActiveIndex++;
            active = getActive();
            active.start();
        }
    }

    @Override
    public void start() {
        System.out.println("series action init");
        mActiveIndex = 0;
        mFinished = mActions.isEmpty();
        if(!mFinished) mActions.get(mActiveIndex).start();
    }

    @Override
    public void stop(boolean interrupted) {
        if(mActions.isEmpty()) return;
        getActive().stop(interrupted);
    }

    @Override
    public boolean isFinished() {
        return mFinished;
    }

    private ActionBase getActive(){
        return mActions.get(mActiveIndex);
    }

    @Override
    public String getID() {
        return mID;
    }
}