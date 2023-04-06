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
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc2023.PlacementStates;
import frc2023.Ports;
import frc2023.Constants.FieldConstants;
import frc2023.Constants.SwerveConstants;
import frc2023.Constants.VisionConstants;
import frc2023.PlacementStates.Node;

import java.util.EnumSet;
import java.util.Optional;

import com.team6328.PoseEstimator.TimestampedVisionUpdate;

public class Limelight extends Subsystem{
    
    private static final InterpolatingTreeMap<Double, Double> kEmptyTreeMap = new InterpolatingTreeMap<>();
            static { kEmptyTreeMap.put(0.0, 0.0); }


    private final NetworkTable mNetworkTable;
    public final PeriodicIO mPeriodicIO = new PeriodicIO();

    private static Limelight mFrontInstance;
    private static Limelight mBackInstance;
    private final Swerve mSwerve = Swerve.getInstance();//this is really bad organization. The swerve should not be referenced in the limelight

    public static Limelight getFrontInstance() {
        if (mFrontInstance == null) {
            mFrontInstance = new Limelight(Ports.frontLimelightName, VisionConstants.distanceFromConeVisionTargetXFrontLimelight, VisionConstants.distanceFromConeVisionTargetYFrontLimelight, 
                                           VisionConstants.frontAprilTagTAToSTDDevs, VisionConstants.kFrontMaxSpeedForVisionUpdateTeleop, VisionConstants.kFrontMaxSpeedForVisionUpdateAuto);
        }
        return mFrontInstance;
    }


    public static Limelight getBackInstance() {
        if (mBackInstance == null) {
            mBackInstance = new Limelight(Ports.backLimelightName, kEmptyTreeMap, kEmptyTreeMap, VisionConstants.backAprilTagTAToSTDDevs, VisionConstants.kBackMaxSpeedForVisionUpdateTeleop, 
                                          VisionConstants.kBackMaxSpeedForVisionUpdateAuto);
        }
        return mBackInstance;
    }


    public class LimelightListener implements TableEventListener{
        @Override
        public void accept(NetworkTable table, String key, NetworkTableEvent event) {
            if(key == "tl"){//if the tl value changes, we assume that all of the values have changed, and we update them. We filter this so that it doesn't update all of the values every time anything changes
                updateValues();
            }
        }
    };


    public static enum VisionTargetType{
        APRILTAG, RETROREFLECTIVE;
    }


    private final InterpolatingTreeMap<Double, Double> txToMetersMap;
    private final InterpolatingTreeMap<Double, Double> tyToMetersMap;
    private final InterpolatingTreeMap<Double, Double> mAprilTagTAToSTDDevs;
    private final String mName;
    private final double mMaxSpeedForVisionUpdateTeleop;
    private final double mMaxSpeedForVisionUpdateAuto;

    private Limelight(String key, InterpolatingTreeMap<Double, Double> txToMetersMap, InterpolatingTreeMap<Double, Double> tyToMetersMap, 
                        InterpolatingTreeMap<Double, Double> aprilTagTAToSTDDevs, double maxSpeedForVisionUpdateTeleop, double maxSpeedForVisionUpdateAuto) {
        mNetworkTable = NetworkTableInstance.getDefault().getTable(key);
        this.txToMetersMap = txToMetersMap;
        this.tyToMetersMap = tyToMetersMap;
        mNetworkTable.addListener("tl", EnumSet.of(NetworkTableEvent.Kind.kValueAll), new LimelightListener());
        mAprilTagTAToSTDDevs = aprilTagTAToSTDDevs;
        mName = key;
        mMaxSpeedForVisionUpdateTeleop = maxSpeedForVisionUpdateTeleop;
        mMaxSpeedForVisionUpdateAuto = maxSpeedForVisionUpdateAuto;
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
        if(mPeriodicIO.currentPipeline == VisionConstants.kAprilTagPipeline) mPeriodicIO.targetType = VisionTargetType.APRILTAG;
        else mPeriodicIO.targetType = VisionTargetType.RETROREFLECTIVE;

        mPeriodicIO.targetValid = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;
        mPeriodicIO.latency = mNetworkTable.getEntry("tl").getDouble(0) / 1000.0 + VisionConstants.kLimelightLatency;
        mPeriodicIO.targetX = mNetworkTable.getEntry("tx").getDouble(0.0);
        mPeriodicIO.targetY = mNetworkTable.getEntry("ty").getDouble(0.0);
        mPeriodicIO.targetArea = mNetworkTable.getEntry("ta").getDouble(0.0);
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


    /**
     *  tells the limelight which vision tape we are targeting. Originally we were going to use the pose of the robot to differentiate
     *  which retroreflective vision target the limelight was seeing, but our pose estimate wasn't accurate enough for that to be reliable. We assume that the 
     *  target the limelight sees is the node the operator has selected. This is pretty accurate since we filter left/right for the vision tape.
     */
    public void setVisionTargetNode(Node target){
        mPeriodicIO.visionTargetNode = target;
    }


    @Override
    public void disable(){}


    public double getLastUpdateTimestamp(){
        return mPeriodicIO.lastUpdatedTimestamp;
    }


    private Optional<TimestampedVisionUpdate> getRobotPoseFromApriltag(){
        //we read the pose from the limelight and check if it doesn't see a target or if we aren't in apriltag mode, don't add a vision measurement
        NetworkTableEntry value = mNetworkTable.getEntry("botpose");
        double[] limelightPose = value.getDoubleArray(new double[0]);
        if(!mPeriodicIO.targetValid || limelightPose.length == 0 || mPeriodicIO.targetType != VisionTargetType.APRILTAG) return Optional.empty();//if there is no target, we don't have a vision update
        if(limelightPose[0] == 0.0 && limelightPose[1] == 0.0) return Optional.empty();

        //We convert the raw limelight values to our mirrored coordinate system
        Pose2d pose = convertLimelightPoseToScreamCoordinates(limelightPose);
       
        //This is our list of filters to discard potentially bad measurements
        if(shouldDiscardApriltagPoseEstimate(pose)) return Optional.empty();

        //if it makes it past the filters, we can use the vision update.
        return Optional.of(new TimestampedVisionUpdate(mPeriodicIO.visionTimestamp, pose, getApriltagSTD_Devs(pose))); 
    }

    private Pose2d convertLimelightPoseToScreamCoordinates(double[] limelightPose){
        if(DriverStation.getAlliance() == Alliance.Blue){// we convert from the limelight coordinate system to our coordinate system.
            return new Pose2d(new Translation2d(-limelightPose[1] + FieldConstants.fieldDimensions.getX()/2, limelightPose[0]), Rotation2d.fromDegrees(limelightPose[2]));
        } else{
            return new Pose2d(new Translation2d(limelightPose[1] - FieldConstants.fieldDimensions.getX()/2, -limelightPose[0]), Rotation2d.fromDegrees(limelightPose[2]));
        } 
    }

    private boolean shouldDiscardApriltagPoseEstimate(Pose2d poseEstimate){
        
        if(!mSwerve.atReference(poseEstimate, VisionConstants.swervePoseErrorToDiscardApriltagMeasurement, Rotation2d.fromDegrees(Double.MAX_VALUE))) return true;// if the vision update is a certain distance from our current pose estimate, we assume it is wrong and filter it out.

        if(mSwerve.getTranslationalSpeed() > mMaxSpeedForVisionUpdateAuto && Timer.getFPGATimestamp() <= 15.0) return true;// auto filtering  //we filter the measuremnt based on the robot speed. If the robot is moving too fast, we don't trust the 
                                                                                                                                                //measurement enough to use it. We have different max speeds for auto and teleop because we trust our swerve odometry more during auto
        else if(mSwerve.getTranslationalSpeed() > mMaxSpeedForVisionUpdateTeleop) return true;// teleop filtering
        
        if(Math.abs(mSwerve.getRotationalSpeed().getRadians()) > 0.4 ) return true;// we also filter based on rotational speed

        return false;
    }

    public Matrix<N3, N1> getApriltagSTD_Devs(Pose2d visionPose){//we change the std devs based on the distance from the tag, measured by the limelight "ta" value.
        double stdDevs = mAprilTagTAToSTDDevs.get(mPeriodicIO.targetArea);
        var output = VecBuilder.fill(stdDevs, stdDevs, Double.MAX_VALUE);
        return output;
    }


    public Matrix<N3, N1> getRetroreflectiveSTD_Devs(){
        return VisionConstants.retroReflectiveMeasurementStandardDeviations;
    }


    /**
     * This is where we measure the robot's pose from the retroreflective vision tape. We have the robot's coordinates for each of its placement positions stored, and we measure the 
     * tx and ty values at that point. We then offset the robot's position in the x/y axes and measure the new limelight tx/ty values with the odometry. We have these values
     * stored in an InterpolatingTreeMap and when we see the vision tape, we plug in the limelight's tx and ty values to get the offset from the placement state. We use these vision
     * measurements for our cone auto placement.
     */
    public Optional<TimestampedVisionUpdate> getPredictedTranslationFromRetroReflectiveTape(Node targetNode){
        if(!mPeriodicIO.targetValid || mPeriodicIO.targetType != VisionTargetType.RETROREFLECTIVE) return Optional.empty();
        if(Math.abs(mSwerve.getRobotPose().getRotation().minus(SwerveConstants.robotForwardAngle).getDegrees()) > VisionConstants.angleThresholdToCountRetroReflectiveMeasurement.getDegrees()) return Optional.empty();

        //checks the target pose, and xOffset and yOffset from target, returns a translation2d for the robot and the stdDevs for the angle are infinite becasue we can't measure angle.
        Pose2d referencePose = PlacementStates.getSwervePlacementPose(targetNode, DriverStation.getAlliance());
        Translation2d offset = new Translation2d(-getXOffsetMetersRetroReflective(), getYOffsetMetersRetroReflective());
        Pose2d pose = new Pose2d(referencePose.getTranslation().plus(offset), referencePose.getRotation());
        
        return Optional.of(new TimestampedVisionUpdate(mPeriodicIO.visionTimestamp, pose, getRetroreflectiveSTD_Devs()));
    }


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


    public enum LedMode {
        PIPELINE, OFF, BLINK, ON
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


    @Override
    public void outputTelemetry() {
        // if(mPeriodicIO.robotPoseFromApriltag.isPresent()) System.out.println("apriltag : " + mPeriodicIO.robotPoseFromApriltag.get().pose) ;
    }

    /**
     * @return robotcentric x offset
     */
    public double getXOffsetMetersRetroReflective(){
        return txToMetersMap.get(mPeriodicIO.targetX);
    }

    
    /**
     * @return robotcentric y offset
     */
    public double getYOffsetMetersRetroReflective(){
        return tyToMetersMap.get(mPeriodicIO.targetY);
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