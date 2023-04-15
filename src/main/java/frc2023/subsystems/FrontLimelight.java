package frc2023.subsystems;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTable.TableEventListener;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc2023.PlacementStates;
import frc2023.Ports;
import frc2023.Constants.FieldConstants;
import frc2023.Constants.SwerveConstants;
import frc2023.Constants.VisionConstants;
import frc2023.Constants.VisionConstants.FrontLimelightConstants;
import frc2023.PlacementStates.Node;

import java.util.EnumSet;
import java.util.Optional;

import com.team6328.PoseEstimator.TimestampedVisionUpdate;

public class FrontLimelight extends Subsystem{

    private final NetworkTable mNetworkTable;
    public final PeriodicIO mPeriodicIO = new PeriodicIO();

    private static FrontLimelight mInstance;
    private final Swerve mSwerve = Swerve.getInstance();//this is really bad organization. The swerve should not be referenced in the limelight

    public static FrontLimelight getInstance() {
        if (mInstance == null) {
            mInstance = new FrontLimelight();
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
        public Node visionTargetNode = Node.NODE1;
        public VisionTargetType targetType;

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
        public double yVisionTapeOffsetMeters;
        public double xVisionTapeOffsetMeters;
        public Rotation2d visionTapeThetaOffset;
        public double targetArea;
        public boolean targetValid;
        public Optional<TimestampedVisionUpdate> robotPoseFromApriltag = Optional.empty();
        public Optional<TimestampedVisionUpdate> robotPoseFromRetroReflective = Optional.empty();

        // OUTPUTS
        public int ledMode = 0; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
        public int camMode = 0; // 0 - vision processing, 1 - driver camera
        public int setPipeline = 0; // 0 - 9
        public int stream = 0; // sets stream layout if another webcam is attached
        public int snapshot = 0; // 0 - stop snapshots, 1 - 2 Hz
    }


    public static enum VisionTargetType{
        APRILTAG, RETROREFLECTIVE;
    }


    public enum LedMode {
        PIPELINE, OFF, BLINK, ON
    }


    private final String mName = Ports.frontLimelightName;

    private FrontLimelight() {
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

        //we read the different network table values
        if(mPeriodicIO.currentPipeline == FrontLimelightConstants.kAprilTagPipeline) mPeriodicIO.targetType = VisionTargetType.APRILTAG;
        else mPeriodicIO.targetType = VisionTargetType.RETROREFLECTIVE;

        mPeriodicIO.targetValid = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;
        mPeriodicIO.latency = mNetworkTable.getEntry("tl").getDouble(0) / 1000.0 + VisionConstants.kLimelightLatency;
        mPeriodicIO.targetX = mNetworkTable.getEntry("tx").getDouble(0.0);
        mPeriodicIO.targetY = mNetworkTable.getEntry("ty").getDouble(0.0);
        mPeriodicIO.targetArea = mNetworkTable.getEntry("ta").getDouble(0.0);
        updateOffsetsFromVisionTape();
        mPeriodicIO.currentLEDMode = (int) mNetworkTable.getEntry("ledMode").getDouble(1.0);
        mPeriodicIO.robotPoseFromApriltag = getRobotPoseFromApriltag();
        mPeriodicIO.robotPoseFromRetroReflective = getPredictedTranslationFromRetroReflectiveTape(mPeriodicIO.visionTargetNode);

        mPeriodicIO.visionTimestamp = Timer.getFPGATimestamp() - mPeriodicIO.latency;//the time that the vision measurement was taken on the camera
        mPeriodicIO.lastUpdatedTimestamp = Timer.getFPGATimestamp();

        //If we are not switching pipelines, and we have a valid vision measurement from the apriltag or retroreflective tape, we add vision measurements to the list in swerve.
         if(!switchingPipelines && mPeriodicIO.robotPoseFromApriltag.isPresent()){
            mSwerve.addVisionMeasurement(mPeriodicIO.robotPoseFromApriltag.get());//vision update from apriltag
        } else if(!switchingPipelines && mPeriodicIO.robotPoseFromRetroReflective.isPresent()){// vision update from retroreflective vision tape. Only used on the front limelight
            mSwerve.addVisionMeasurement(mPeriodicIO.robotPoseFromRetroReflective.get());
        }  

    }


    private Optional<TimestampedVisionUpdate> getRobotPoseFromApriltag(){
        //we read the pose from the limelight and check if it doesn't see a target or if we aren't in apriltag mode, don't add a vision measurement
        NetworkTableEntry value = mNetworkTable.getEntry("botpose");
        double[] limelightPose = value.getDoubleArray(new double[0]);
        if(!mPeriodicIO.targetValid || limelightPose.length == 0 || mPeriodicIO.targetType != VisionTargetType.APRILTAG) return Optional.empty();//if there is no target, we don't have a vision update
        if(limelightPose[0] == 0.0 && limelightPose[1] == 0.0){
            return Optional.empty();
        } 

        //We convert the raw limelight values to our mirrored coordinate system
        Pose2d pose = convertLimelightPoseToScreamCoordinates(limelightPose);
        
        //This is our list of filters to discard potentially bad measurements
        if(shouldDiscardApriltagPoseEstimate(pose)) return Optional.empty();

        //if it makes it past the filters, we can use the vision update.
        return Optional.of(new TimestampedVisionUpdate(mPeriodicIO.visionTimestamp, pose, getApriltagSTD_Devs(pose))); 
    }

    private Pose2d convertLimelightPoseToScreamCoordinates(double[] limelightPose){
        if(DriverStation.getAlliance() == Alliance.Blue){// we convert from the limelight coordinate system to our coordinate system.
            return new Pose2d(new Translation2d(-limelightPose[1] + FieldConstants.fieldDimensions.getX()/2, limelightPose[0]), Rotation2d.fromDegrees(limelightPose[5]-90));
        } else{
            return new Pose2d(new Translation2d(limelightPose[1] - FieldConstants.fieldDimensions.getX()/2, -limelightPose[0]), Rotation2d.fromDegrees(limelightPose[5]+90));
        } 
    }

    private boolean shouldDiscardApriltagPoseEstimate(Pose2d poseEstimate){
        
        if(!mSwerve.atReference(poseEstimate, FrontLimelightConstants.swervePoseErrorToDiscardApriltagMeasurement, Rotation2d.fromDegrees(Double.MAX_VALUE))) return true;// if the vision update is a certain distance from our current pose estimate, we assume it is wrong and filter it out.

        if(mSwerve.getTranslationalSpeed() > FrontLimelightConstants.kMaxSpeedForVisionUpdateAuto && Timer.getFPGATimestamp() <= 15.0) return true;// auto filtering  //we filter the measuremnt based on the robot speed. If the robot is moving too fast, we don't trust the 
                                                                                                                                                //measurement enough to use it. We have different max speeds for auto and teleop because we trust our swerve odometry more during auto
        else if(mSwerve.getTranslationalSpeed() > FrontLimelightConstants.kMaxSpeedForVisionUpdateTeleop) return true;// teleop filtering
        
        if(Math.abs(mSwerve.getRotationalSpeed().getRadians()) > 0.4 ) return true;// we also filter based on rotational speed //TODO extract to constant

        return false;
    }

    public Matrix<N3, N1> getApriltagSTD_Devs(Pose2d visionPose){//we change the std devs based on the distance from the tag, measured by the limelight "ta" value.
        double translationSTDDevs = FrontLimelightConstants.aprilTagTAToTranslationSTDDevs.get(mPeriodicIO.targetArea);
        double angleStdDevs = FrontLimelightConstants.aprilTagTAToAngleSTDDevs.get(mPeriodicIO.targetArea);
        var output = VecBuilder.fill(translationSTDDevs, translationSTDDevs, angleStdDevs);
        return output;
    }


    /**
     * This is where we measure the robot's pose from the retroreflective vision tape. We have the robot's coordinates for each of its placement positions stored, and we measure the 
     * tx and ty values at that point. We then offset the robot's position in the x/y axes and measure the new limelight tx/ty values with the odometry. We have these values
     * stored in an InterpolatingTreeMap and when we see the vision tape, we plug in the limelight's tx and ty values to get the offset from the placement state. We use these vision
     * measurements for our cone auto placement.
     */
    public Optional<TimestampedVisionUpdate> getPredictedTranslationFromRetroReflectiveTape(Node targetNode){
        if(!mPeriodicIO.targetValid || mPeriodicIO.targetType != VisionTargetType.RETROREFLECTIVE) return Optional.empty();
        if(Math.abs(mSwerve.getRobotPose().getRotation().minus(SwerveConstants.robotForwardAngle).getDegrees()) > FrontLimelightConstants.angleThresholdToCountRetroReflectiveMeasurement.getDegrees()) return Optional.empty();

        //checks the target pose, and xOffset and yOffset from target, returns a translation2d for the robot and the stdDevs for the angle are infinite becasue we can't measure angle.
        Pose2d referencePose = PlacementStates.getSwervePlacementPose(targetNode, DriverStation.getAlliance());
        Translation2d offset = new Translation2d(mPeriodicIO.xVisionTapeOffsetMeters, mPeriodicIO.yVisionTapeOffsetMeters);
        Pose2d pose = new Pose2d(referencePose.getTranslation().plus(offset), referencePose.getRotation().plus(mPeriodicIO.visionTapeThetaOffset));

        return Optional.of(new TimestampedVisionUpdate(mPeriodicIO.visionTimestamp, pose, getRetroreflectiveSTD_Devs()));
    }


    public Matrix<N3, N1> getRetroreflectiveSTD_Devs(){
        double stdDevs = FrontLimelightConstants.retroReflectiveTAToTranslationSTDDevScalarMap.get(mPeriodicIO.targetArea);
        // double angleSTDDevs = FrontLimelightConstants.retroReflectiveTAToAngleSTDDevScalarMap.get(mPeriodicIO.targetArea);//TODO either fix or remove, right now the scalar is 1 so it doesnt matter anyways
        return FrontLimelightConstants.retroReflectiveMeasurementStandardDeviations.times(stdDevs);
    }


    /**
     *  tells the limelight which vision tape we are targeting. Originally we were going to use the pose of the robot to differentiate
     *  which retroreflective vision target the limelight was seeing, but our pose estimate wasn't accurate enough for that to be reliable. We assume that the 
     *  target the limelight sees is the node the operator has selected. This is pretty accurate since we filter left/right for the vision tape.
     */
    public void setVisionTargetNode(Node target){
        mPeriodicIO.visionTargetNode = target;
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


    public double readTLDirectlyFromNetworkTable(){
        return mNetworkTable.getEntry("tl").getDouble(0.0);
    }

    
    public void updateOffsetsFromVisionTape(){
    if(!mPeriodicIO.targetValid) return;
        final double limelightUp = 0.770;//TODO extract to constants
        final double visionTapeUp = 0.6096;
        final Rotation2d limelightPitch = Rotation2d.fromDegrees(0.0000001);//TODO clean up this jank NaN fix
        final double limelightDistanceFromBumper = 0.102+0.4315734464756843;
        final double limelightRightOffset = 0.17217-0.3-0.0535;


        Rotation2d angleToGoal = Rotation2d.fromDegrees(limelightPitch.getDegrees() + mPeriodicIO.targetY);

        double distanceFromLimelightToGoalMeters = (visionTapeUp - limelightUp)/angleToGoal.getTan();
        mPeriodicIO.visionTapeThetaOffset = Rotation2d.fromDegrees(mPeriodicIO.targetX);
        mPeriodicIO.yVisionTapeOffsetMeters = distanceFromLimelightToGoalMeters - limelightDistanceFromBumper;
        System.out.println("robotThetaOffset: " + mPeriodicIO.visionTapeThetaOffset.getDegrees());//TODO test back LL as well
        mPeriodicIO.xVisionTapeOffsetMeters = (mPeriodicIO.visionTapeThetaOffset.getTan()*distanceFromLimelightToGoalMeters)-limelightRightOffset;
    }


    @Override
    public void disable(){}


    public double getLastUpdateTimestamp(){
        return mPeriodicIO.lastUpdatedTimestamp;
    }


    @Override
    public void outputTelemetry() {

    }
    

    // /**
    //  * @return robotcentric x offset
    //  */
    // public double getXOffsetMetersRetroReflective(){
    //     return FrontLimelightConstants.distanceFromConeVisionTargetXMap.get(mPeriodicIO.targetX);
    // }

    
    // /**
    //  * @return robotcentric y offset
    //  */
    // public double getYOffsetMetersRetroReflective(){
    //     return FrontLimelightConstants.distanceFromConeVisionTargetYMap.get(mPeriodicIO.targetY);
    // }


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