package frc2023.shuffleboard;

import java.util.ArrayList;

import frc2023.Constants;
import frc2023.shuffleboard.tabs.*;
public class ShuffleboardTabManager {

    // Adds more tabs to use when debugging
    public final boolean mDebug = Constants.includeDebugTabs;

    private static ShuffleboardTabManager mInstance; 

    public static ShuffleboardTabManager getInstance() {
        if (mInstance == null) {
            mInstance = new ShuffleboardTabManager();
        }
        return mInstance;
    }


    private final ArrayList<ShuffleboardTabBase> mTabs = new ArrayList<ShuffleboardTabBase>();

    public final MatchTab matchTab = MatchTab.getInstance();
    public final SwerveTab swerveTab = SwerveTab.getInstance();
    public final TestTab testTab = TestTab.getInstance();
    public final ArmTab armTab = ArmTab.getInstance();
    public final IntakeTab intakeTab = IntakeTab.getInstance();
    public final GripperTab gripperTab = GripperTab.getInstance();

    public ShuffleboardTabManager() {
        mTabs.add(matchTab);
        if (mDebug) {
            mTabs.add(swerveTab);
            mTabs.add(armTab);
            mTabs.add(testTab);
            mTabs.add(intakeTab);
            mTabs.add(gripperTab);
        } else {
           
        }

        for(ShuffleboardTabBase tab: mTabs) {
            tab.createEntries();
        }
    }

    public void update() {
        for (ShuffleboardTabBase tab : mTabs) {
            tab.update();
        }
    }
}