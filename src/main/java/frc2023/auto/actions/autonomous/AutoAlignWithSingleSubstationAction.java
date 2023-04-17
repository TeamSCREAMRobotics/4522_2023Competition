package frc2023.auto.actions.autonomous;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc2023.Constants.PlacementConstants;
import frc2023.auto.actions.lib.ActionBase;
import frc2023.controlboard.ControlBoard;
import frc2023.field.MirroredRotation;
import frc2023.subsystems.Arm;
import frc2023.subsystems.BackLimelight;
import frc2023.subsystems.Gripper;
import frc2023.subsystems.Swerve;
import frc2023.subsystems.Gripper.GripperState;

public class AutoAlignWithSingleSubstationAction extends ActionBase{

    private final Arm mArm = Arm.getInstance();
    private final Gripper mGripper = Gripper.getInstance();
    private final Swerve mSwerve = Swerve.getInstance();
    private final BackLimelight mBackLimelight = BackLimelight.getInstance();
    private final ControlBoard mControlBoard = ControlBoard.getInstance();
    private final Alliance mAlliance;
    private final boolean mZeroOnStart;
    private boolean mNotZeroedYet = true;

    public AutoAlignWithSingleSubstationAction(Alliance alliance, boolean zeroOnStart){
        mAlliance = alliance;
        mZeroOnStart = zeroOnStart;
    }

    @Override
    public void start() {
        // if(mZeroOnStart){
        //     mSwerve.resetTranslation(PlacementConstants.swerveZeroBeforeSubstationPoint.getPoint(mAlliance));
        //  }
    }

    @Override
    public void run() {
        
        if(mControlBoard.getArmManualOverride()){
            mArm.setPercentOutput(mControlBoard.getPivotPO(), mControlBoard.getTelescopePO());
        } else{
            mArm.retrieveCone();  

        }
        mGripper.open();
        if(mNotZeroedYet && mZeroOnStart && mBackLimelight.getTargetValid()){
            mSwerve.resetTranslation(PlacementConstants.swerveZeroBeforeSubstationPoint.getPoint(mAlliance));
            mNotZeroedYet = false;
        }

        if(mBackLimelight.getTargetValid()){
            mSwerve.setAlignWithSingleSubstation(PlacementConstants.singleSubstationConeRetrievalPoint.get(mAlliance));
        } else{//normal driving if we don't see the tag

            Translation2d swerveTranslation = mControlBoard.getSwerveTranslation();

            if(mControlBoard.getSlowMode()) {
                swerveTranslation = swerveTranslation.times(mControlBoard.getSlowModeTranslationScalar());
            }

            mSwerve.driveAndFaceAngle(swerveTranslation, MirroredRotation.get(180, mAlliance), false);

        }
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
