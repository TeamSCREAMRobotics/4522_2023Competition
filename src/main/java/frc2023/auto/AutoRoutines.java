package frc2023.auto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc2023.PlacementStates;
import frc2023.Constants.ArmConstants;
import frc2023.Constants.FieldConstants;
import frc2023.PlacementStates.Level;
import frc2023.PlacementStates.Node;
import frc2023.auto.actions.autonomous.ResetRobotPoseAction;
import frc2023.auto.actions.autonomous.RunArmAction;
import frc2023.auto.actions.autonomous.autoPlacement.ArmAutoPlaceAction;
import frc2023.auto.actions.lib.ActionBase;
import frc2023.auto.actions.lib.SeriesAction;
import frc2023.auto.actions.lib.WaitAction;
import frc2023.auto.modes.VariableSpeedTrajectory.TrajectorySpeed;
import frc2023.field.MirroredTranslation;
import frc2023.field.MirroredPose;

public class AutoRoutines {

    public static ActionBase getTestRoutine(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE1, alliance))
        );
    }

    
    public static ActionBase startX_DoNothing(MirroredPose startPoint, Alliance alliance){
        return new ResetRobotPoseAction(startPoint.get(alliance));
    }

    
    public static ActionBase startX_ScoreCone(MirroredPose startPoint, Alliance alliance){
        return new SeriesAction(
            startX_DoNothing(startPoint, alliance),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            new RunArmAction(ArmConstants.Setpoints.kConeRetrieval)
        );
    }

        
    public static ActionBase startX_ScoreCube(MirroredPose startPoint, Alliance alliance){
        return new SeriesAction(
            startX_DoNothing(startPoint, alliance),
            AutoSegments.shootCubeHigh()
        );
    }

    
    public static ActionBase start1_1Cone_2Cube(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE1, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            AutoSegments.node1ToGamePiece1(alliance, false),
            AutoSegments.gamePiece1ToScoreNode2(alliance),
            AutoSegments.node2ToGamePiece2(alliance, TrajectorySpeed.MEDIUM, false),
            AutoSegments.gamePiece2ToScoreNode2(alliance)
        );
    }


    public static ActionBase start1_1Cone_1Cube_AutoBalance(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE1, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            AutoSegments.node1ToGamePiece1(alliance, false),
            AutoSegments.gamePiece1ToScoreNode2(alliance),
            AutoSegments.autoBalanceFromNodeSide(FieldConstants.chargeStationNode1SideTarget, alliance, true, true)
        );
    }


    public static ActionBase start1_1Cone_2Cube_Coast(Alliance alliance){
        return new SeriesAction(
            start1_1Cone_2Cube(alliance),   
            AutoSegments.coastWhileDisabledAction(new MirroredTranslation(-0.3, 3.3), 0.55, 14.1, alliance) 
        );
    }

    
    public static ActionBase start1_1Cone_1Cube_1PoopShoot_AutoBalance(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE1, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            AutoSegments.node1ToGamePiece1(alliance, false),
            AutoSegments.gamePiece1ToScoreNode2(alliance),
            AutoSegments.node2ToGamePiece2(alliance, TrajectorySpeed.FAST, true),
            AutoSegments.poopShootWhileBalanceFromGamePieceSide(FieldConstants.chargeStationNode1SideTarget, alliance)
        );
    }


    public static ActionBase start1_1Cone_1Cube_2Poopshoot_Coast(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE1, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            AutoSegments.node1ToGamePiece1(alliance, true),
            AutoSegments.gamePiece1ToScoreNode2(alliance),
            AutoSegments.node2ToGamePiece2(alliance, TrajectorySpeed.MEDIUM, true),
            AutoSegments.gamePiece2ToShoot5_1(alliance),
            AutoSegments.shoot5ToGamePiece3_1(alliance),
            AutoSegments.gamePiece3ToShoot7_1(alliance),
            AutoSegments.coastWhileDisabledAction(new MirroredTranslation(-3.3, 0.5), 0.45, 14.1, alliance)
        );
    }


    public static ActionBase start1_1Cone_1Cube_2Poopshoot_AutoBalance(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE1, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            AutoSegments.node1ToGamePiece1(alliance, true),
            AutoSegments.gamePiece1ToScoreNode2(alliance),
            AutoSegments.node2ToGamePiece2(alliance, TrajectorySpeed.MEDIUM, true),
            AutoSegments.gamePiece2ToShoot5_1(alliance),
            AutoSegments.shoot5ToGamePiece3_1(alliance),
            AutoSegments.poopShootWhileBalanceFromGamePieceSide(FieldConstants.chargeStationNode9SideTarget, alliance)
        );
    }


    public static ActionBase start1_1Cone_3PoopShoot(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE1, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            AutoSegments.node1ToGamePiece1(alliance, true),
            AutoSegments.gamePiece1ToShoot3_1(alliance),
            AutoSegments.shoot3ToGamePiece2_1(alliance),
            AutoSegments.gamePiece2ToShoot5_1(alliance),
            AutoSegments.shoot5ToGamePiece3_1(alliance),
            AutoSegments.gamePiece3ToShoot7_1(alliance)
        );
    }


    public static ActionBase start1_1Cone_3Poopshoot_AutoBalance(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE1, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            AutoSegments.node1ToGamePiece1(alliance, true),
            AutoSegments.gamePiece1ToShoot3_1(alliance),
            AutoSegments.shoot3ToGamePiece2_1(alliance),
            AutoSegments.gamePiece2ToShoot5_1(alliance),
            AutoSegments.shoot5ToGamePiece3_1(alliance),
            AutoSegments.poopShootWhileBalanceFromGamePieceSide(FieldConstants.chargeStationNode9SideTarget, alliance)
        );
    }


    public static ActionBase start1_1Cone_3PoopShoot_Coast(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE1, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            AutoSegments.node1ToGamePiece1(alliance, true),
            AutoSegments.gamePiece1ToShoot3_1(alliance),
            AutoSegments.shoot3ToGamePiece2_1(alliance),
            AutoSegments.gamePiece2ToShoot5_1(alliance),
            AutoSegments.shoot5ToGamePiece3_1(alliance),
            AutoSegments.gamePiece3ToShoot7_1(alliance),
            AutoSegments.coastWhileDisabledAction(new MirroredTranslation(-3.3, 0.5), 0.45, 14.1, alliance)
        );
    }

    //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

    public static ActionBase start4_1Cone_AutoBalance(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE4, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            AutoSegments.autoBalanceFromNodeSide(FieldConstants.chargeStationNode1SideTarget, alliance, true, true)
        );
    }


    public static ActionBase start4_1Cone_AutoBalanceOverAndBack(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE4, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            AutoSegments.middleDriveOverChargeStationAndDriveBack(FieldConstants.chargeStationNode1SideTarget, alliance, true, true)
        );
    }


    public static ActionBase start5_1Cube_AutoBalance(MirroredTranslation chargeStationTarget, Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE5, alliance)),
            AutoSegments.shootCubeHigh(),
            AutoSegments.autoBalanceFromNodeSide(chargeStationTarget, alliance, true, true)
        );
    }


    public static ActionBase start5_1Cube_AutoBalanceOverAndBack(MirroredTranslation chargeStationTarget, Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE5, alliance)),
            AutoSegments.shootCubeHigh(),
            AutoSegments.middleDriveOverChargeStationAndDriveBack(chargeStationTarget, alliance, true, true)
        );
    }


    public static ActionBase start6_1Cone_AutoBalance(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE6, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            AutoSegments.autoBalanceFromNodeSide(FieldConstants.chargeStationNode9SideTarget, alliance, true, true)
        );
    }


    public static ActionBase start6_1Cone_AutoBalanceOverAndBack(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE6, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            AutoSegments.middleDriveOverChargeStationAndDriveBack(FieldConstants.chargeStationNode9SideTarget, alliance, true, true)
        );
    }


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
        

    public static ActionBase start9_1Cone_1Cube(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE9, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            new WaitAction(ArmConstants.waitAfterPlace9),
            AutoSegments.node9ToGamePiece4(alliance, TrajectorySpeed.SLOW, false),
            AutoSegments.gamePiece4ToScoreNode8SLOW(alliance)
        );
    }

        
    public static ActionBase start9_1Cone_1Cube_AutoBalance(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE9, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            new WaitAction(ArmConstants.waitAfterPlace9),
            AutoSegments.node9ToGamePiece4(alliance, TrajectorySpeed.SLOW, false),
            AutoSegments.gamePiece4ToScoreNode8SLOW(alliance),
            AutoSegments.autoBalanceFromNodeSide(FieldConstants.chargeStationNode9SideTarget, alliance, true, true)
        );
    }


    public static ActionBase start9_1Cone_1Cube_Coast(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE9, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            new WaitAction(ArmConstants.waitAfterPlace9),
            AutoSegments.node9ToGamePiece4(alliance, TrajectorySpeed.SLOW, false),
            AutoSegments.gamePiece4ToScoreNode8SLOW(alliance),
            AutoSegments.coastBack(alliance)
        );
    }

    
    public static ActionBase start9_1Cone_2Cube(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE9, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            new WaitAction(ArmConstants.waitAfterPlace9),
            AutoSegments.node9ToGamePiece4(alliance, TrajectorySpeed.MEDIUM, false),
            AutoSegments.gamePiece4ToScoreNode8FAST(alliance),
            AutoSegments.node8ToGamePiece3(alliance, false),
            AutoSegments.gamePiece3ToScoreNode8(alliance, TrajectorySpeed.FAST)
        );
    }

      
    public static ActionBase start9_1Cone_2Cube_CoastBack(Alliance alliance){
        return new SeriesAction(
            start9_1Cone_2Cube(alliance),
            AutoSegments.coastBack(alliance)
        );
    }


    public static ActionBase start9_1Cone_1Cube_GrabCubeDontShoot(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE9, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            new WaitAction(ArmConstants.waitAfterPlace9),
            AutoSegments.node9ToGamePiece4(alliance, TrajectorySpeed.SLOW, false),
            AutoSegments.gamePiece4ToScoreNode8SLOW(alliance),
            AutoSegments.node8ToGamePiece3(alliance,false)
        );
    }


    public static ActionBase start9_1Cone_1Cube_1PoopShoot(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE9, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            new WaitAction(ArmConstants.waitAfterPlace9),
            AutoSegments.node9ToGamePiece4(alliance, TrajectorySpeed.MEDIUM, false),
            AutoSegments.gamePiece4ToScoreNode8SLOW(alliance),
            AutoSegments.node8ToGamePiece3(alliance, true),
            AutoSegments.gamePiece3ToShoot8_9(alliance)
        );
    }


    public static ActionBase start9_1Cone_2PoopShoot(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE9, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            new WaitAction(ArmConstants.waitAfterPlace9),
            AutoSegments.node9ToGamePiece4(alliance, TrajectorySpeed.MEDIUM, true),
            AutoSegments.gamePiece4ToShoot8_9(alliance),
            AutoSegments.shoot8ToGamePiece3_9(alliance),
            AutoSegments.gamePiece3ToShoot5_9(alliance)
        );
    }


    public static ActionBase start9_1Cone_1Cube_1PoopShoot_AutoBalance(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE9, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            new WaitAction(ArmConstants.waitAfterPlace9),
            AutoSegments.node9ToGamePiece4(alliance, TrajectorySpeed.MEDIUM, false),
            AutoSegments.gamePiece4ToScoreNode8FAST(alliance),
            AutoSegments.node8ToGamePiece3(alliance, true),
            AutoSegments.poopShootWhileBalanceFromGamePieceSide(FieldConstants.chargeStationNode9SideTarget, alliance)
        );
    }

    
    public static ActionBase start9_1Cone_1Cube_GrabCubeDontShoot_AutoBalance(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE9, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            new WaitAction(ArmConstants.waitAfterPlace9),
            AutoSegments.node9ToGamePiece4(alliance, TrajectorySpeed.MEDIUM, false),
            AutoSegments.gamePiece4ToScoreNode8FAST(alliance),
            AutoSegments.node8ToGamePiece3(alliance, false),
            AutoSegments.autoBalanceFromGamePieceSide(FieldConstants.chargeStationNode9SideTarget, alliance, false, true, true)
        );
    }


    public static ActionBase start9_1Cone_1Cube_1PoopShoot_Coast(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE9, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            new WaitAction(ArmConstants.waitAfterPlace9),
            AutoSegments.node9ToGamePiece4(alliance, TrajectorySpeed.MEDIUM, false),
            AutoSegments.gamePiece4ToScoreNode8SLOW(alliance),
            AutoSegments.node8ToGamePiece3(alliance, true),
            AutoSegments.gamePiece3ToShoot5_9(alliance),
            AutoSegments.coastWhileDisabledAction(new MirroredTranslation(-3.3, 0.5), 0.45, 14.1, alliance)
        );
    }


    public static ActionBase start9_1Cone_2PoopShoot_AutoBalance(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE9, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            new WaitAction(ArmConstants.waitAfterPlace9),
            AutoSegments.node9ToGamePiece4(alliance, TrajectorySpeed.MEDIUM, true),
            AutoSegments.gamePiece4ToShoot8_9(alliance),
            AutoSegments.shoot8ToGamePiece3_9(alliance),
            AutoSegments.poopShootWhileBalanceFromGamePieceSide(FieldConstants.chargeStationNode9SideTarget, alliance)
        );
    }


    public static ActionBase start9_1Cone_1Cube_2PoopShoot_Coast(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE9, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            new WaitAction(ArmConstants.waitAfterPlace9),
            AutoSegments.node9ToGamePiece4(alliance, TrajectorySpeed.MEDIUM, false),
            AutoSegments.gamePiece4ToScoreNode8FAST(alliance),
            AutoSegments.node8ToGamePiece3(alliance, true),
            AutoSegments.gamePiece3ToShoot5_9(alliance),
            AutoSegments.shoot5ToGamePiece2_9(alliance),
            AutoSegments.gamePiece2ToShoot4_9(alliance),
            AutoSegments.coastWhileDisabledAction(new MirroredTranslation(-3.3, 0.5), 0.45, 14.1, alliance)
        );
    }


    public static ActionBase start9_1Cone_3Poopshoot(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE9, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            new WaitAction(ArmConstants.waitAfterPlace9),
            AutoSegments.node9ToGamePiece4(alliance, TrajectorySpeed.MEDIUM, true),
            AutoSegments.gamePiece4ToShoot8_9(alliance),
            AutoSegments.shoot8ToGamePiece3_9(alliance),
            AutoSegments.gamePiece3ToShoot9_9(alliance),
            AutoSegments.shoot9ToGamePiece2_9(alliance),
            AutoSegments.gamePiece2ToShoot8_9(alliance)
        );
    }


    public static ActionBase start9_1Cone_3Poopshoot_LeaveCommunity(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE9, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            new WaitAction(ArmConstants.waitAfterPlace9),
            AutoSegments.node9ToGamePiece4(alliance, TrajectorySpeed.MEDIUM, true),
            AutoSegments.gamePiece4ToShoot8_9(alliance),
            AutoSegments.shoot8ToGamePiece3_9(alliance),
            AutoSegments.gamePiece3ToShoot9_9(alliance),
            AutoSegments.shoot9ToGamePiece2_9(alliance),
            AutoSegments.gamePiece2ToShoot8_9(alliance),
            AutoSegments.shoot8ToLeaveCommunity(alliance)
        );
    }


    public static ActionBase start9_1Cone_3PoopShoot_AutoBalance(Alliance alliance){
        return new SeriesAction(
            new ResetRobotPoseAction(PlacementStates.getSwervePlacementPose(Node.NODE9, alliance)),
            new ArmAutoPlaceAction(PlacementStates.getArmPlacementStateForAuto(Level.TOP)),
            new WaitAction(ArmConstants.waitAfterPlace9),
            AutoSegments.node9ToGamePiece4(alliance, TrajectorySpeed.MEDIUM, true),
            AutoSegments.gamePiece4ToShoot8_9(alliance),
            AutoSegments.shoot8ToGamePiece3_9(alliance),
            AutoSegments.gamePiece3ToShoot5_9(alliance),
            AutoSegments.shoot5ToGamePiece2_9(alliance),
            AutoSegments.poopShootWhileBalanceFromGamePieceSide(FieldConstants.chargeStationNode1SideTarget, alliance)
        );
    }
}
