package frc2023.auto.actions.autonomous.autoPlacement;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc2023.PlacementStates;
import frc2023.Constants.SwerveConstants;
import frc2023.PlacementStates.Level;
import frc2023.PlacementStates.Node;
import frc2023.auto.actions.lib.ActionBase;
import frc2023.controlboard.ControlBoard;
import frc2023.subsystems.Arm;
import frc2023.subsystems.Gripper;
import frc2023.subsystems.Intake;
import frc2023.subsystems.Limelight;
import frc2023.subsystems.Swerve;

public class LimelightAutoScore extends ActionBase{

    private final Arm mArm = Arm.getInstance();
    private final Swerve mSwerve = Swerve.getInstance();
    private final Gripper mGripper = Gripper.getInstance();
    private final Limelight mFrontLimelight = Limelight.getFrontInstance();
    private final ControlBoard mControlBoard = ControlBoard.getInstance();
    private final Intake mIntake = Intake.getInstance();

    private final Node mNode;
    private final Level mLevel;
    private final Translation2d mArmTranslation;
    private final int mPipeline;
    private final Pose2d mPlacementState;
    private double mLastTimestampNotAtTarget = 0.0;
    private Alliance mAlliance;
    private final boolean mZeroOnStart;

    public LimelightAutoScore(Node node, Level level, boolean zeroOnStart){
        mNode = node;
        mLevel = level;
        mAlliance = DriverStation.getAlliance();
        mArmTranslation  = PlacementStates.getArmPlacementState(mLevel);
        mPipeline = PlacementStates.getVisionPipeline(mNode, DriverStation.getAlliance());
        mPlacementState = PlacementStates.getSwervePlacementPose(mNode, DriverStation.getAlliance());
        mZeroOnStart = zeroOnStart;
    }

    @Override
    public void start() {
        mGripper.close();
        if(mZeroOnStart){
           mSwerve.resetTranslation(PlacementStates.getSwerveBackupBeforePlaceState(mNode, mAlliance).getTranslation());
        }
    }

    private boolean successfulAutoPlace = false;

    @Override
    public void run() {
        mFrontLimelight.setPipeline(mPipeline);
        mSwerve.setVisionSnap(mPlacementState);
              
        if(mNode.isCone()){
            mArm.setPlacementPosition(mLevel);
            mIntake.forceRetract();

            if(!(mArm.atTargetPosition(mArmTranslation) && mSwerve.atReference(mPlacementState, SwerveConstants.visionXTolerance, SwerveConstants.visionYTolerance, SwerveConstants.visionThetaTolerance))){
                mLastTimestampNotAtTarget = Timer.getFPGATimestamp();//measures the last time that the arm wasn't at its target. Used for gripper auto place logic
            }
        } else if(mNode.isCube()){
            mArm.retrieveCube();
            mIntake.shootCube(mLevel);
        }

        //Manual Override stuff
        final boolean manualOverride = mControlBoard.getArmManualOverride();

        /* if(!manualOverride && Timer.getFPGATimestamp()-mLastTimestampNotAtTarget > 0.1){         //this is the logic for the gripper to 
                                                                                                    auto place if the robot has been in the right spot for a 
                                                                                                    certain amount of time. We commented this out because it was 
                                                                                                    safer to have the operator do it manually and there wasn't 
                                                                                                    really any drawback
            mGripper.placeCone();
            successfulAutoPlace = true;
        }  */
        
        if(manualOverride){
            mArm.setPercentOutput(mControlBoard.getPivotPO(), mControlBoard.getTelescopePO());
        } 

        if(mControlBoard.getOpenGripper()){
            mGripper.open();
            successfulAutoPlace = true;
        } else if(mControlBoard.getCloseGripper()) mGripper.close();
    }

    @Override
    public void stop(boolean interrupted) {
        mSwerve.disable();
        mFrontLimelight.setPipeline(0);//when finished, switch limelight back to apriltags
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public String getID() {
        return "Limelight Auto Place: Node:" + (mNode.index+1) + ", Level: " + (mLevel.index+1);
    }
}