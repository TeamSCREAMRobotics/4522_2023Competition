package frc2023.auto.actions.autonomous;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc2023.Constants.PlacementConstants;
import frc2023.auto.actions.lib.ActionBase;
import frc2023.subsystems.Arm;
import frc2023.subsystems.Gripper;
import frc2023.subsystems.Swerve;
import frc2023.subsystems.Gripper.GripperState;

public class AutoAlignWithSingleSubstationAction extends ActionBase{

    private final Arm mArm = Arm.getInstance();
    private final Gripper mGripper = Gripper.getInstance();
    private final Swerve mSwerve = Swerve.getInstance();
    private final Alliance mAlliance;
    private final boolean mZeroOnStart;

    public AutoAlignWithSingleSubstationAction(Alliance alliance, boolean zeroOnStart){
        mAlliance = alliance;
        mZeroOnStart = zeroOnStart;
    }

    @Override
    public void start() {
        if(mZeroOnStart){
            mSwerve.resetTranslation(PlacementConstants.swerveZeroBeforeSubstationPoint.getPoint(mAlliance));
         }
    }

    @Override
    public void run() {
        mArm.retrieveCone();  
        mGripper.open();
        mSwerve.snapToPose(PlacementConstants.singleSubstationConeRetrievalPoint.get(mAlliance));
    }

    @Override
    public void stop(boolean interrupted) {
        System.out.println("arm auto place stop");
        mArm.disable();
        mGripper.open();
    }


    @Override
    public boolean isFinished() {
        
        return false;
    }

    @Override
    public String getID() {
        return this.getClass().getName();
    }
}
