package frc2023;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc2023.Constants.ArmConstants;
import frc2023.Constants.FieldConstants;
import frc2023.Constants.PlacementConstants;
import frc2023.Constants.VisionConstants;
import frc2023.field.MirroredTranslation;
import frc2023.field.MirroredPose;

public class PlacementStates {
    public static Translation2d getArmPlacementState(Level level){
        switch(level){
            case HYBRID:
                return ArmConstants.Setpoints.kHybridCone;
            case MIDDLE:
                return ArmConstants.Setpoints.kMiddleCone;
            default:
                DriverStation.reportError("error in getArmPlacementState", false);
            case TOP:
                return ArmConstants.Setpoints.kTopCone;
        }
    }

    /**
     * @return the arm translation for placemnt at the beginning of auto. These setpoints are different than our normal swerve is lined up, so the arm can slam
     *  the cone down aggressively. This means the arm telescope must be extended less because the cone is propelled out by the centrifugal force.
     */
    public static Translation2d getArmPlacementStateForAuto(Level level){
        switch(level){
            case HYBRID:
                return ArmConstants.Setpoints.kHybridConeAutoStart;
            case MIDDLE:
                return ArmConstants.Setpoints.kMiddleConeAutoStart;
            default:
                DriverStation.reportError("error in getArmPlacementStateForAutoBeginning", false);
            case TOP:
                return ArmConstants.Setpoints.kTopConeAutoStart;
        }
    }

    /**
     * @return returns the pose that the robot should be in to auto place at the selected node
     */
    public static Pose2d getSwervePlacementPose(Node node, Alliance alliance){
        return getSwervePlacementPose(node).get(alliance);
    }

    /**
     * @return returns the pose that the robot should be in to auto place at the selected node
     */
    public static MirroredPose getSwervePlacementPose(Node node){
        return PlacementConstants.swervePlacementStates[node.index];
    }

    /**
     * @return returns the translation that the robot should be in to auto place at the selected node
     */
    public static MirroredTranslation getSwervePlacementTranslation(Node node){
        return getSwervePlacementPose(node).getMirroredTranslation();
    }

    
    /**
     * @return returns the pose that the robot should be in to auto place at the selected node for auto. We have the robot shoot cubes from slightly further back during auto
     */
    public static MirroredPose getSwervePlacementPoseAuto(Node node){
        return PlacementConstants.swerveAutoPlacementStates[node.index];
    }
    
    /**
     * @return returns the translation that the robot should be in to auto place at the selected node for auto. We have the robot shoot cubes from slightly further back during auto
     */
    public static MirroredTranslation getSwervePlacementTranslationAuto(Node node){
        return getSwervePlacementPoseAuto(node).getMirroredTranslation();
    }

    /**
     * @return Returns a pose that is slightly backed up from the placement state. This is so that if we are moving horizontally in the community, we don't run into the nodes
     */
    public static Pose2d getSwerveBackupBeforePlaceState(Node node, Alliance alliance){
        return PlacementConstants.swerveBackupBeforePlaceStates[node.index].get(alliance);
    }

    /**
     * @return returns which limelight pipeline we need to select for the given node. 
     * 
     * <p> We have a pipeline for apriltags(for cubes) and retroreflective pipelines that
     * bias towards the left and right sides to help filter which target the robot goes towards.
     */
    public static int getVisionPipeline(Node node, Alliance alliance){
        if(node.isCube()) return VisionConstants.kAprilTagPipeline;
        
        boolean leftPipeline = ((node == Node.NODE1 || node == Node.NODE3 || node == Node.NODE6) && alliance == Alliance.Red) || ((node == Node.NODE9 || node == Node.NODE7 || node == Node.NODE4) && alliance == Alliance.Blue);
        //logic for right pipeline: boolean rightPipeline = ((node == Node.NODE1 || node == Node.NODE3 || node == Node.NODE6) && alliance == Alliance.Blue) || ((node == Node.NODE9 || node == Node.NODE7 || node == Node.NODE4) && alliance == Alliance.Red)
        return (leftPipeline? VisionConstants.kConeLeftPipeline : VisionConstants.kConeRightPipeline);
    }
////////////////////////////////////////  enums related field elements  ////////////////////////////////////
    public enum GamePieceType{
        CONE, CUBE;
        public boolean isCube(){
            return this == CUBE;
        }

        public boolean isCone(){
            return this == CONE;
        }
    }

    public static enum Level{
        HYBRID(0), MIDDLE(1), TOP(2);

        public int index;
        private Level(int index){
            this.index = index;
        }
    }

    public static enum Node{
        NODE1(0), NODE2(1), NODE3(2), NODE4(3), NODE5(4), NODE6(5), NODE7(6), NODE8(7), NODE9(8);
        public int index;
        private Node(int index){
            this.index = index;
        }

        public GamePieceType getType(){
            if(isCube()) return GamePieceType.CUBE;
            else return GamePieceType.CONE;
        }

        public double getXValue(Alliance alliance){
            return getLocation(Level.HYBRID, alliance).getX();//the level is arbitrarily hybrid because we don't care about the y axis
        }

        public Translation2d getLocation(Level level, Alliance alliance){
            return FieldConstants.nodeLocations[index][level.index].getPoint(alliance);//the index is 0 because we don't care about the y axis
        }

        public boolean isCube(){
            return this == NODE2 || this == NODE5 || this == NODE8;
        }

        public boolean isCone(){
            return !isCube();
        }

        public static Node get(int index){
            for(Node node : Node.values()) if(node.index == index) return node;
            DriverStation.reportWarning("bad index in Node", false);
            return NODE1;
        }

        public MirroredTranslation getVisionTargetLocation(){
            return FieldConstants.visionTargetLocations[index];
        }
    }
}