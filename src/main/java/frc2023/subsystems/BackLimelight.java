package frc2023.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc2023.PlacementStates;
import frc2023.Ports;
import frc2023.Constants.PlacementConstants;
import frc2023.Constants.SwerveConstants;
import frc2023.Constants.VisionConstants;
import frc2023.Constants.VisionConstants.BackLimelightConstants;
import frc2023.PlacementStates.Node;
import frc2023.field.MirroredRotation;

import java.util.EnumSet;
import java.util.Optional;

import com.team6328.PoseEstimator.TimestampedVisionUpdate;

public class BackLimelight extends Subsystem{
    
    private final NetworkTable mNetworkTable;
    public final PeriodicIO mPeriodicIO = new PeriodicIO();

    private static BackLimelight mInstance;
    private final Swerve mSwerve = Swerve.getInstance();//this is really bad organization. The swerve should not be referenced in the BackLimelight


    public static BackLimelight getInstance() {
        if (mInstance == null) {
            mInstance = new BackLimelight();
        }
        return mInstance;
    }


    public class LimelightListener implements TableEventListener{
        @Override
        public void accept(NetworkTable table, String key, NetworkTableEvent event) {
            if(key == "tl"){//if the tl value changes, we assume that all of the values have changed, and we update them. We filter this so that it doesn't update all of the values every time anything changes
                updateValues();
            }
        }
    };


    public static class PeriodicIO {

        // INPUTS

        public double lastUpdatedTimestamp;
        public double lastPipelineChangeTimestamp;

        public int currentPipeline;
        public boolean pipelineChanged;
        public double timeUntilCanUpdatePose;

        public double latency;
        public double visionTimestamp;
        public int currentLEDMode;
        public double targetX;
        public double targetY;
        public double yOffsetMeters;
        public double xOffsetMeters;
        public double targetArea;
        public boolean targetValid;
        public Optional<TimestampedVisionUpdate> robotPoseFromSubstationTag = Optional.empty();

        // OUTPUTS
        public int ledMode = 0; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
        public int camMode = 0; // 0 - vision processing, 1 - driver camera
        public int setPipeline = 0; // 0 - 9
        public int stream = 0; // sets stream layout if another webcam is attached
        public int snapshot = 0; // 0 - stop snapshots, 1 - 2 Hz
        public Rotation2d thetaOffset;
    }


    public enum LedMode {
        PIPELINE, OFF, BLINK, ON
    }


    private final String mName = Ports.backLimelightName;

    private BackLimelight() {
        mNetworkTable = NetworkTableInstance.getDefault().getTable(mName);
    
        mNetworkTable.addListener("tl", EnumSet.of(NetworkTableEvent.Kind.kValueAll), new LimelightListener());
    }


    public synchronized void updateValues() {
        //checks if the pipeline has changed and updates the measured pipeline
        int newPipeline = (int) mNetworkTable.getEntry("pipeline").getDouble(0);
        if(newPipeline != mPeriodicIO.currentPipeline) mPeriodicIO.pipelineChanged = true;
        else mPeriodicIO.pipelineChanged = false;
        mPeriodicIO.currentPipeline = newPipeline;
        
        if(mPeriodicIO.pipelineChanged){//we record the last time the pipeline has changed
            mPeriodicIO.lastPipelineChangeTimestamp = Timer.getFPGATimestamp();
        }

        //if we just switched pipelines, we have to wait for a small duration before we can read correct vision measurements
        boolean switchingPipelines = (Timer.getFPGATimestamp()-mPeriodicIO.lastPipelineChangeTimestamp  < VisionConstants.limelightSwitchPipelineDelay);

        mPeriodicIO.targetValid = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;
        mPeriodicIO.latency = mNetworkTable.getEntry("tl").getDouble(0) / 1000.0 + VisionConstants.kLimelightLatency;
        mPeriodicIO.targetX = mNetworkTable.getEntry("tx").getDouble(0.0);
        mPeriodicIO.targetY = mNetworkTable.getEntry("ty").getDouble(0.0);
        mPeriodicIO.targetArea = mNetworkTable.getEntry("ta").getDouble(0.0);
        mPeriodicIO.currentLEDMode = (int) mNetworkTable.getEntry("ledMode").getDouble(1.0);

        updateOffsetsFromSubstationTag();

        mPeriodicIO.visionTimestamp = Timer.getFPGATimestamp() - mPeriodicIO.latency;//the time that the vision measurement was taken on the camera
        mPeriodicIO.lastUpdatedTimestamp = Timer.getFPGATimestamp();

        mPeriodicIO.robotPoseFromSubstationTag = getPredictedTranslationFromSubstationTag(DriverStation.getAlliance());

        if(!switchingPipelines && mPeriodicIO.robotPoseFromSubstationTag.isPresent()){
            mSwerve.addVisionMeasurement(mPeriodicIO.robotPoseFromSubstationTag.get());//vision update from apriltag
        }

        // System.out.println(mPeriodicIO.xOffsetMeters + "  TY: " + mPeriodicIO.yOffsetMeters);
    }

    @Override
    public synchronized void writeOutputs() {
        if(mPeriodicIO.currentLEDMode != mPeriodicIO.currentLEDMode || mPeriodicIO.currentPipeline != mPeriodicIO.setPipeline) {
            mNetworkTable.getEntry("ledMode").setNumber(mPeriodicIO.currentLEDMode);
            mNetworkTable.getEntry("camMode").setNumber(mPeriodicIO.camMode);
            mNetworkTable.getEntry("pipeline").setNumber(mPeriodicIO.setPipeline);
            mNetworkTable.getEntry("stream").setNumber(mPeriodicIO.stream);
            mNetworkTable.getEntry("snapshot").setNumber(mPeriodicIO.snapshot);
        }
    }

    

    /**
     * This is where we measure the robot's pose from the retroreflective vision tape. We have the robot's coordinates for each of its placement positions stored, and we measure the 
     * tx and ty values at that point. We then offset the robot's position in the x/y axes and measure the new limelight tx/ty values with the odometry. We have these values
     * stored in an InterpolatingTreeMap and when we see the vision tape, we plug in the limelight's tx and ty values to get the offset from the placement state. We use these vision
     * measurements for our cone auto placement.
     */
    public Optional<TimestampedVisionUpdate> getPredictedTranslationFromSubstationTag(Alliance alliance){
        if(!mPeriodicIO.targetValid) return Optional.empty();
        if(Math.abs(mSwerve.getRobotPose().getRotation().minus(MirroredRotation.get(180, alliance)).getDegrees()) > BackLimelightConstants.angleThresholdToCountSubstationTagMeasurement.getDegrees()) return Optional.empty();

        //checks the target pose, and xOffset and yOffset from target, returns a translation2d for the robot and the stdDevs for the angle are infinite becasue we can't measure angle.
        Pose2d referencePose = PlacementConstants.singleSubstationConeRetrievalPoint.get(alliance);
        Translation2d offset = new Translation2d(-mPeriodicIO.yOffsetMeters, mPeriodicIO.xOffsetMeters);
        Pose2d pose = new Pose2d(referencePose.getTranslation().plus(offset), referencePose.getRotation().minus(mPeriodicIO.thetaOffset));

        return Optional.of(new TimestampedVisionUpdate(mPeriodicIO.visionTimestamp, pose, getSubstationTagSTD_Devs()));
    }

 
    public Matrix<N3, N1> getSubstationTagSTD_Devs(){
        return BackLimelightConstants.substationTagMeasurementStandardDeviations;
    }


    public void updateOffsetsFromSubstationTag(){
        if(!mPeriodicIO.targetValid) return;

        final double limelightUp = 0.770;
        final double substationUp = 1.346;
        final Rotation2d limelightPitch = Rotation2d.fromDegrees(30);
        final double limelightDistanceFromBumper = 0.587637970798052 - 0.015+0.04;
        final double limelightRightOffset = 0.17217 - 0.03;

        Rotation2d angleToGoal = Rotation2d.fromDegrees(limelightPitch.getDegrees() + mPeriodicIO.targetY);

        double distanceFromLimelightToGoalMeters = (substationUp - limelightUp)/angleToGoal.getTan();
        mPeriodicIO.thetaOffset = Rotation2d.fromDegrees(mPeriodicIO.targetX);
        mPeriodicIO.yOffsetMeters = distanceFromLimelightToGoalMeters - limelightDistanceFromBumper;
        mPeriodicIO.xOffsetMeters = (mPeriodicIO.thetaOffset.getTan()*distanceFromLimelightToGoalMeters)-limelightRightOffset;
        // System.out.println(" x: " + mPeriodicIO.xOffsetMeters + "  y: " + mPeriodicIO.yOffsetMeters + " theta: " + mPeriodicIO.thetaOffset);

    }



    public double readTLDirectlyFromNetworkTable(){
        return mNetworkTable.getEntry("tl").getDouble(0.0);
    }


    @Override
    public void disable(){}


    public double getLastUpdateTimestamp(){
        return mPeriodicIO.lastUpdatedTimestamp;
    }


    @Override
    public void outputTelemetry() {

    }


    @Override
    public void stop() {
    
    }


    public int getPipeline() {
        return mPeriodicIO.currentPipeline;
    }


    public void setPipeline(int newPipeline){
        mPeriodicIO.setPipeline = newPipeline;
    }


    public boolean getTargetValid() {
        return mPeriodicIO.targetValid;
    }
}