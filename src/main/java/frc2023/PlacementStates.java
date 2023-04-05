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
     * @return the arm translation for an auto place. This is only for the start locations that the robot beins at during auto.
     */
    public static Translation2d getArmPlacementStateForAutoBeginning(Level level){
        switch(level){
            case HYBRID:
                return ArmConstants.Setpoints.kHybridConeAutoStart;
            case MIDDLE:
                return ArmConstants.Setpoints.kMiddleConeAutoStart;
            default:
                DriverStation.reportError("error1 in getArmPlacementState", false);
            case TOP:
                return ArmConstants.Setpoints.kTopConeAutoStart;
        }
    }

    public static Pose2d getSwervePlacementPose(Node node, Alliance alliance){
        return getSwervePlacementPose(node).get(alliance);
    }

    public static MirroredPose getSwervePlacementPose(Node node){
        return PlacementConstants.swervePlacementStates[node.index];
    }

    public static MirroredTranslation getSwervePlacementTranslation(Node node){
        return getSwervePlacementPose(node).getMirroredTranslation();
    }

    
    public static MirroredPose getSwervePlacementPoseAuto(Node node){
        return PlacementConstants.swerveAutoPlacementStates[node.index];
    }
    
    public static MirroredTranslation getSwervePlacementTranslationAuto(Node node){
        return getSwervePlacementPoseAuto(node).getMirroredTranslation();
    }

    public static Pose2d getSwerveBackupBeforePlaceState(Node node, Alliance alliance){
        return PlacementConstants.swerveBackupBeforePlaceStates[node.index].get(alliance);
    }

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
            return getLocation(Level.HYBRID, alliance).getX();//the level is hybrid because we don't care about the y axis
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