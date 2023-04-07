package frc2023.auto;


import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc2023.Constants.ArmConstants;
import frc2023.Constants.FieldConstants;
import frc2023.Constants.PlacementConstants;
import frc2023.Constants.SwerveConstants;
import frc2023.Constants.IntakeConstants.ShooterConstants;
import frc2023.auto.actions.autonomous.AutoBalanceAction;
import frc2023.auto.actions.autonomous.DriveAction;
import frc2023.auto.actions.autonomous.FollowTrajectoryAction;
import frc2023.auto.actions.autonomous.MoveToPoseAction;
import frc2023.auto.actions.autonomous.RunArmAction;
import frc2023.auto.actions.autonomous.RunIntakeAction;
import frc2023.auto.actions.autonomous.SetIntakeAction;
import frc2023.auto.actions.autonomous.WaitUntilTrajectoryProgressAction;
import frc2023.auto.actions.autonomous.autoPlacement.PoopShootPositionAction;
import frc2023.auto.actions.lib.ActionBase;
import frc2023.auto.actions.lib.EmptyAction;
import frc2023.auto.actions.lib.InstantAction;
import frc2023.auto.actions.lib.ParallelAction;
import frc2023.auto.actions.lib.RaceAction;
import frc2023.auto.actions.lib.RunUntilConditionAction;
import frc2023.auto.actions.lib.SeriesAction;
import frc2023.auto.actions.lib.WaitAction;
import frc2023.auto.actions.lib.WaitUntilConditionAction;
import frc2023.auto.modes.VariableKinematicsTrajectory.TrajectorySpeed;
import frc2023.field.MirroredTranslation;
import frc2023.field.MirroredRotation;
import frc2023.subsystems.Swerve;
import frc2023.subsystems.Intake.IntakeState;

public class AutoSegments {

    private static Swerve mSwerve = Swerve.getInstance();

	public static ActionBase shootCubeHigh() {
		return new RaceAction(
            new RunIntakeAction(IntakeState.SHOOT_CUBE_HIGH_AUTO),
            new WaitAction(1.2)
        );
	}

    public static ActionBase node1ToGamePiece1(Alliance alliance, boolean poopShootNext){
        return new SeriesAction(
            new RaceAction(
                new FollowTrajectoryAction(Trajectories.node1ToGamePiece1.getTrajectory(alliance, TrajectorySpeed.MEDIUM), MirroredRotation.get(-90, alliance), 2.3),
                new SeriesAction(
                    new RaceAction(
                        new RunArmAction(ArmConstants.Setpoints.kAutoWithoutGamepiece),
                        new WaitUntilTrajectoryProgressAction(.45)
                    ),
                    new ParallelAction(
                        new RunIntakeAction(poopShootNext? IntakeState.INTAKE_FOR_POOPSHOOT : IntakeState.INAKE_AUTO),
                        new RunArmAction(ArmConstants.Setpoints.kPoopShoot)
                    )
                )
            )
        );
    }

    public static ActionBase gamePiece1ToScoreNode2(Alliance alliance){
        return new SeriesAction(
            new RaceAction(
                new FollowTrajectoryAction(Trajectories.gamePiece1ToNode2.getTrajectory(alliance, TrajectorySpeed.FAST), SwerveConstants.robotForwardAngle, 1.8),
                new RunArmAction(ArmConstants.Setpoints.kAutoWithGamePiece),
                new SeriesAction(
                    new SeriesAction(
                        new RaceAction(
                            new RunIntakeAction(IntakeState.EJECT_ONLY_LOWER_CONVEYOR),
                            new WaitAction(0.50)
                        ),
                        new RaceAction(
                            new RunIntakeAction(IntakeState.FORCE_RETRACT),
                            new WaitUntilTrajectoryProgressAction(.85)
                        )
                    ),
                    new RunIntakeAction(IntakeState.SHOOT_CUBE_HIGH_AUTO)
                )
            ),
            new RaceAction(
                new RunIntakeAction(IntakeState.SHOOT_CUBE_HIGH_AUTO),
                new WaitAction(.6)
            ),
            new SetIntakeAction(IntakeState.FORCE_RETRACT)
        );
    }


    public static ActionBase node2ToGamePiece2(Alliance alliance, TrajectorySpeed speed, boolean poopShootNext){
        return new SeriesAction(
            new RaceAction(
                new FollowTrajectoryAction(Trajectories.node2ToGamePiece2.getTrajectory(alliance, speed), MirroredRotation.get(58-180, alliance), 3.0),
                new SeriesAction(
                    new RaceAction(
                        new WaitUntilTrajectoryProgressAction(.5),
                        new RunArmAction(ArmConstants.Setpoints.kAutoWithoutGamepiece)
                    ),
                    new ParallelAction(
                        new RunIntakeAction(poopShootNext? IntakeState.INTAKE_FOR_POOPSHOOT : IntakeState.INAKE_AUTO),
                        new RunArmAction(ArmConstants.Setpoints.kPoopShoot)
                    )
                )
            )
        );
    }


    public static ActionBase gamePiece2ToScoreNode2(Alliance alliance){
        return new SeriesAction(
            new RaceAction(   
                new FollowTrajectoryAction(Trajectories.gamePiece2ToNode2.getTrajectory(alliance, TrajectorySpeed.FAST), SwerveConstants.robotForwardAngle, 2.3),
                new RunArmAction(ArmConstants.Setpoints.kAutoWithGamePiece),
                new SeriesAction(
                    new SeriesAction(
                        new RaceAction(
                            new RunIntakeAction(IntakeState.EJECT_ONLY_LOWER_CONVEYOR),
                            new WaitAction(0.50)
                        ),
                        new RaceAction(
                            new RunIntakeAction(IntakeState.FORCE_RETRACT),
                            new WaitUntilTrajectoryProgressAction(.7)
                        )
                    ),
                    new RunIntakeAction(IntakeState.SHOOT_CUBE_MID_AUTO)
                )
            ),
            new RaceAction(
                new RunIntakeAction(IntakeState.SHOOT_CUBE_MID_AUTO),
                new WaitAction(0.6)
            ),
            new SetIntakeAction(IntakeState.FORCE_RETRACT)
        );
    }



    public static ActionBase node9ToGamePiece4(Alliance alliance, TrajectorySpeed speed, boolean poopShootNext){
        return new SeriesAction(
            new RaceAction(
                new FollowTrajectoryAction(Trajectories.node9ToGamePiece4.getTrajectory(alliance, speed), MirroredRotation.get(-90, alliance), 2.3),
                new SeriesAction(
                    new RaceAction(
                        new RunArmAction(ArmConstants.Setpoints.kAutoWithoutGamepiece),
                        new WaitUntilTrajectoryProgressAction(.50)
                    ),
                    new ParallelAction(
                        new RunArmAction(ArmConstants.Setpoints.kPoopShoot),
                        new RunIntakeAction(poopShootNext? IntakeState.INTAKE_FOR_POOPSHOOT : IntakeState.INAKE_AUTO)
                    )
                )
            )
        );
    }



    public static ActionBase gamePiece4ToScoreNode8SLOW(Alliance alliance){// we have a slow and fast segment for this because the timers get messed up
        return new SeriesAction(
            new RaceAction(
                new FollowTrajectoryAction(Trajectories.gamePiece4ToNode8.getTrajectory(alliance, TrajectorySpeed.SLOW), SwerveConstants.robotForwardAngle, 2.3),
                new RunArmAction(ArmConstants.Setpoints.kAutoWithGamePiece),
                new SeriesAction(
                    new SeriesAction(
                        new RaceAction(
                            new RunIntakeAction(IntakeState.EJECT_ONLY_LOWER_CONVEYOR),
                            new WaitAction(0.20)
                        ),
                        new RaceAction(
                            new RunIntakeAction(IntakeState.FORCE_RETRACT),
                            new WaitUntilTrajectoryProgressAction(0.85)
                        )
                    ),
                    new RunIntakeAction(IntakeState.SHOOT_CUBE_HIGH_AUTO)
                )
            ),
            new RaceAction(
                new RunIntakeAction(IntakeState.SHOOT_CUBE_HIGH_AUTO),
                new WaitAction(.7)
            ),
            new SetIntakeAction(IntakeState.FORCE_RETRACT)
        );
    }


    public static ActionBase gamePiece4ToScoreNode8FAST(Alliance alliance){// we have a slow and fast segment for this because the timers get messed up
        return new SeriesAction(
            new RaceAction(
                new FollowTrajectoryAction(Trajectories.gamePiece4ToNode8.getTrajectory(alliance, TrajectorySpeed.FAST), SwerveConstants.robotForwardAngle, 2.3),
                new RunArmAction(ArmConstants.Setpoints.kAutoWithGamePiece),
                new SeriesAction(
                    new SeriesAction(
                        new RaceAction(
                            new RunIntakeAction(IntakeState.EJECT_ONLY_LOWER_CONVEYOR),
                            new WaitAction(0.20)
                        ),
                        new RaceAction(
                            new RunIntakeAction(IntakeState.FORCE_RETRACT),
                            new WaitUntilTrajectoryProgressAction(0.75)//TODO instead of waiting for a percentage, this should be waiting until a specific amount of time is left in the trajectory because the intake speed is constant regardless of the trajectory speed, so it will require different times for different speeds
                        )
                    ),
                    new RunIntakeAction(IntakeState.SHOOT_CUBE_HIGH_AUTO)
                )
            ),
            new RaceAction(
                new RunIntakeAction(IntakeState.SHOOT_CUBE_HIGH_AUTO),
                new WaitAction(.5)
            ),
            new SetIntakeAction(IntakeState.FORCE_RETRACT)
        );
    }


    public static ActionBase node8ToGamePiece3(Alliance alliance, boolean poopShootNext){
        return new SeriesAction(
            new RaceAction(
                new FollowTrajectoryAction(Trajectories.node8ToGamePiece3.getTrajectory(alliance, TrajectorySpeed.MEDIUM),  MirroredRotation.get(-55, alliance), 3.0),
                new SeriesAction(
                    new RaceAction(
                        new RunArmAction(ArmConstants.Setpoints.kAutoWithoutGamepiece),
                        new WaitUntilTrajectoryProgressAction(.55)
                    ),
                    new ParallelAction(
                        new RunIntakeAction(poopShootNext? IntakeState.INTAKE_FOR_POOPSHOOT : IntakeState.INAKE_AUTO),
                        new RunArmAction(ArmConstants.Setpoints.kPoopShoot)
                    )
                )
            )
        );
    }



    public static ActionBase gamePiece3ToScoreNode8(Alliance alliance){
        return new SeriesAction(
            new RaceAction(   
                new FollowTrajectoryAction(Trajectories.gamePiece3ToNode8.getTrajectory(alliance, TrajectorySpeed.SLOW), SwerveConstants.robotForwardAngle, 2.3),
                new RunArmAction(ArmConstants.Setpoints.kAutoWithGamePiece),
                new SeriesAction(
                    new RaceAction(
                        new RunIntakeAction(IntakeState.FORCE_RETRACT),
                        new WaitUntilTrajectoryProgressAction(.70)                        
                    ),
                    new RunIntakeAction(IntakeState.SHOOT_CUBE_MID_AUTO)
                )
            ),
            new RaceAction(
                new RunIntakeAction(IntakeState.SHOOT_CUBE_MID_AUTO),
                new WaitAction(1)
            ),
            new SetIntakeAction(IntakeState.FORCE_RETRACT)
        );
    }


    public static ActionBase gamePiece1ToShoot3_1(Alliance alliance){
        return new SeriesAction(
            new PoopShootPositionAction(PlacementConstants.shootLocation3.getPoint(alliance),  ShooterConstants.kDefaultAutoShootDuration),
            new SetIntakeAction(IntakeState.FORCE_RETRACT)
        );
    }


    public static ActionBase gamePiece2ToShoot5_1(Alliance alliance){
        return new SeriesAction(
            new PoopShootPositionAction(PlacementConstants.shootLocation5.getPoint(alliance),  ShooterConstants.kDefaultAutoShootDuration),
            new SetIntakeAction(IntakeState.FORCE_RETRACT)
        );
    }


    public static ActionBase gamePiece3ToShoot7_1(Alliance alliance){
        return new SeriesAction(
            new PoopShootPositionAction(PlacementConstants.shootLocation7.getPoint(alliance), ShooterConstants.kDefaultAutoShootDuration),
            new SetIntakeAction(IntakeState.FORCE_RETRACT)
        );
    }


    public static ActionBase gamePiece4ToShoot6_1(Alliance alliance){
        return new SeriesAction(
            new PoopShootPositionAction(PlacementConstants.shootLocation6.getPoint(alliance), ShooterConstants.kDefaultAutoShootDuration),
            new SetIntakeAction(IntakeState.FORCE_RETRACT)
        );
    }

    
    public static ActionBase gamePiece4ToShoot8_9(Alliance alliance){
        return new SeriesAction(
            new PoopShootPositionAction(PlacementConstants.shootLocation8.getPoint(alliance),  ShooterConstants.kDefaultAutoShootDuration),
            new SetIntakeAction(IntakeState.FORCE_RETRACT)
        );
    }

 
    public static ActionBase gamePiece3ToShoot5_9(Alliance alliance){
        return new SeriesAction(
            new PoopShootPositionAction(PlacementConstants.shootLocation5.getPoint(alliance), 0.55),
            new SetIntakeAction(IntakeState.FORCE_RETRACT)
        );
    } 


    public static ActionBase gamePiece2ToShoot4_9(Alliance alliance){
        return new SeriesAction(
            new PoopShootPositionAction(PlacementConstants.shootLocation4.getPoint(alliance), ShooterConstants.kDefaultAutoShootDuration),
            new SetIntakeAction(IntakeState.FORCE_RETRACT)
        );
    }


    public static ActionBase shoot3ToGamePiece2_1(Alliance alliance){
        return new SeriesAction(
            new RaceAction(
                new FollowTrajectoryAction(Trajectories.shoot3ToGamePiece2_1.getTrajectory(alliance, TrajectorySpeed.FAST), MirroredRotation.get(60-180, alliance), 2.8),
                new RunArmAction(ArmConstants.Setpoints.kAutoWithGamePiece),
                new SeriesAction(
                    new WaitUntilTrajectoryProgressAction(0.00),
                    new RunIntakeAction(IntakeState.INTAKE_FOR_POOPSHOOT)
                )
            )
        );
    }


    public static ActionBase shoot5ToGamePiece3_1(Alliance alliance){
        return new SeriesAction(
            new RaceAction(
                new FollowTrajectoryAction(Trajectories.shoot5ToGamePiece3_1.getTrajectory(alliance, TrajectorySpeed.FAST),  FieldConstants.stagingMark3.getPoint(alliance), 2.8, SwerveConstants.robotBackwardAngle),
                new RunArmAction(ArmConstants.Setpoints.kAutoWithGamePiece),
                new SeriesAction(
                    new WaitUntilTrajectoryProgressAction(0.00),
                    new RunIntakeAction(IntakeState.INTAKE_FOR_POOPSHOOT)
                )
            )
        );
    }


    public static ActionBase shoot7ToGamePiece4_1(Alliance alliance){
        return new SeriesAction(
            new RaceAction(
                new FollowTrajectoryAction(Trajectories.shoot7ToGamePiece4_1.getTrajectory(alliance, TrajectorySpeed.FAST),  FieldConstants.stagingMark4.getPoint(alliance), 2.8, SwerveConstants.robotBackwardAngle),
                new RunArmAction(ArmConstants.Setpoints.kAutoWithGamePiece),
                new SeriesAction(
                    new WaitUntilTrajectoryProgressAction(0.00),
                    new RunIntakeAction(IntakeState.INTAKE_FOR_POOPSHOOT)
                )
            )
        );
    }


    public static ActionBase shoot8ToGamePiece3_9(Alliance alliance){
        return new SeriesAction(
            new RaceAction(
                new FollowTrajectoryAction(Trajectories.shoot8ToGamePiece3_9.getTrajectory(alliance, TrajectorySpeed.SLOW),  MirroredRotation.get(-60, alliance), 2.8),
                new RunArmAction(ArmConstants.Setpoints.kAutoWithGamePiece),
                new SeriesAction(
                    new WaitUntilTrajectoryProgressAction(0.00),
                    new RunIntakeAction(IntakeState.INTAKE_FOR_POOPSHOOT)
                )
            )
        );
    }


    public static ActionBase shoot5ToGamePiece2_9(Alliance alliance){
        return new SeriesAction(
            new RaceAction(
                new FollowTrajectoryAction(Trajectories.shoot5ToGamePiece2_9.getTrajectory(alliance, TrajectorySpeed.SLOW),  MirroredRotation.get(-65, alliance), 2.8),
                new RunArmAction(ArmConstants.Setpoints.kAutoWithGamePiece),
                new SeriesAction(
                    new WaitUntilTrajectoryProgressAction(0.00),
                    new RunIntakeAction(IntakeState.INTAKE_FOR_POOPSHOOT)
                )
            )
        );
    }

    
   

    public static ActionBase autoBalanceFromNodeSide(MirroredTranslation chargeStationTarget, Alliance alliance, boolean bringArmIn, boolean bringIntakeIn){
        return new ParallelAction(
            optionalBringInSubsystemsAction(bringArmIn, bringIntakeIn),
            new SeriesAction(
                new MoveToPoseAction(new Pose2d(chargeStationTarget.plus(new Translation2d(0, -1)).getPoint(alliance), SwerveConstants.robotForwardAngle), false),
                driveForwardUntilChargeStation(),
                new AutoBalanceAction(false)
            )
        );
    }


    public static ActionBase autoBalanceFromGamePieceSide(MirroredTranslation chargeStationTarget, Alliance alliance, boolean robotForward, boolean bringArmIn, boolean bringIntakeIn) {
        return new ParallelAction(
            optionalBringInSubsystemsAction(bringArmIn, bringIntakeIn),
            new SeriesAction(
                new MoveToPoseAction(new Pose2d(chargeStationTarget.plus(new Translation2d(0, 1)).getPoint(alliance), (robotForward? SwerveConstants.robotForwardAngle : SwerveConstants.robotBackwardAngle)), false),
                driveBackUntilChargeStation(),
                new AutoBalanceAction(!robotForward)
            )
        );
    }


    public static ActionBase optionalBringInSubsystemsAction(boolean bringArmIn, boolean bringIntakeIn){
        return new ParallelAction(
            (bringArmIn? new RunArmAction(ArmConstants.Setpoints.kConeRetrieval) : new EmptyAction()),
            (bringIntakeIn? new RunIntakeAction(IntakeState.FORCE_RETRACT) : new EmptyAction())
        );
    }


    public static ActionBase middleDriveOverChargeStationAndDriveBack(MirroredTranslation chargeStationTarget, Alliance alliance, boolean bringArmIn, boolean bringIntakeIn) {
        return new ParallelAction(
            optionalBringInSubsystemsAction(bringArmIn, bringIntakeIn),
            new SeriesAction(
                new MoveToPoseAction(new Pose2d(chargeStationTarget.plus(new Translation2d(0, -1)).getPoint(alliance), SwerveConstants.robotForwardAngle), false),
                new DriveAction(new Translation2d(0, 1.2), 0, false, 2.4),
                driveBackUntilChargeStation(),
                new AutoBalanceAction(false)
            )
        );
    }


    public static ActionBase driveForwardUntilChargeStation(){
        return  new SeriesAction(
                
            new RunUntilConditionAction(
                        new DriveAction(new Translation2d(0, SwerveConstants.kRunOntoChargeStationSpeed), 0, false, 3.0)
                            ) {
                @Override
                public boolean condition() {
                    return Math.abs(mSwerve.getPitch().getDegrees()) > SwerveConstants.kPitchToConsiderOnChargeStation;
                }

                @Override
                public String getID() {
                    return "driveForwardUntilChargeStation";
                }  
            },
            new DriveAction(new Translation2d(0, SwerveConstants.kRunOntoChargeStationSpeed), 0, false, SwerveConstants.kDriveAfterPitchThresholdMetAutoBalance)                
            
        );
    }


    public static ActionBase driveBackUntilChargeStation(){
        return new SeriesAction(
            new RunUntilConditionAction(
                    new SeriesAction(
                        new DriveAction(new Translation2d(0, -SwerveConstants.kRunOntoChargeStationSpeed), 0, false, 3.0),
                        new DriveAction(new Translation2d(0, SwerveConstants.kBackUpChargeStationJammedSpeed), 0, false, 0.5),//The last 2 actions are timeouts. If the charging  station gets stuck, we back up and try again.
                        new DriveAction(new Translation2d(0, -SwerveConstants.kRunOntoChargeStationSpeed-1.0), 0, false, 3.0)
                    )
                    ) {

                @Override
                public boolean condition() {
                    return Math.abs(mSwerve.getPitch().getDegrees()) > SwerveConstants.kPitchToConsiderOnChargeStation;
                }

                @Override
                public String getID() {
                    return "driveBackUntilChargeStation";
                }
            },
            new DriveAction(new Translation2d(0, -SwerveConstants.kRunOntoChargeStationSpeed), 0, false, SwerveConstants.kDriveAfterPitchThresholdMetAutoBalance)
        );
    }


    public static ActionBase coastBack(Alliance alliance) {
        return coastWhileDisabledAction(new MirroredTranslation(0, 3.3), 0.55, 14.1, alliance);
    }


    private static ActionBase instantEnterBrakeModeAction(){
        return new InstantAction() {
            @Override
            public void start() {
                Swerve.getInstance().setNeutralMode(NeutralMode.Coast, NeutralMode.Brake);
            }

            @Override
            public String getID() {
                return "instantBRAKEMODEACTION";
            }
        };
    }


    public static ActionBase coastWhileDisabledAction(MirroredTranslation driveInput, double duration, double startTimeFGPA, Alliance alliance){
        return new SeriesAction(
            new WaitUntilConditionAction() {
                @Override
                public boolean condition() {
                    return Timer.getFPGATimestamp() >= startTimeFGPA;
                }

                @Override
                public String getID() {
                    return "wait for coastWhileDisabledAction";
                }
            },
            instantEnterBrakeModeAction(),
            new DriveAction(driveInput.getPoint(alliance), 0, false, duration)
        );
    }


    public static ActionBase poopShootWhileBalanceFromGamePieceSide(MirroredTranslation chargeStationTarget, Alliance alliance){
        return new ParallelAction(
            new SeriesAction(
                new WaitAction(0.45),
                new RunIntakeAction(IntakeState.POOP_SHOOT_FROM_CHARGE_LINE_AUTO)
            ),
            AutoSegments.autoBalanceFromGamePieceSide(chargeStationTarget, alliance, false, true, false)
        );
    }
}
