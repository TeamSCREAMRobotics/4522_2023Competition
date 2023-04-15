package frc2023;


import com.team254.CanDeviceId;
import com.team254.SwerveSetpointGenerator.KinematicLimits;
import com.team4522.lib.pid.MotionMagicConstants;
import com.team4522.lib.pid.PIDConstants;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj.DriverStation;

import frc2023.Constants.SwerveModuleConstants.DefaultConfig;
import frc2023.Constants.SwerveModuleConstants.ModuleLocation;
import frc2023.Ports.ModuleIDs;
import frc2023.field.MirroredTranslation;
import frc2023.field.MirroredPose;

public class Constants {

	public static final double kSubsystemPeriodSeconds = 0.02;
	public static final int kSubsystemPeriodMilliseconds = (int) (kSubsystemPeriodSeconds * 1000.0);
	public static final double kOdometryPeriodSeconds = 0.01;
	public static final int kOdometryPeriodMilliseconds = (int) (kOdometryPeriodSeconds * 1000.0);
	public static final double kTelemetryPeriodSeconds = 0.40;
	public static final int kTelemetryPeriodMilliseconds = (int) (kTelemetryPeriodSeconds * 1000.0);
	public static final double kUpdatePIDsFromShuffleboardPeriodSeconds = 3.00;
	public static final int kUpdatePIDsFromShuffleboardPeriodMilliseconds = (int) (kUpdatePIDsFromShuffleboardPeriodSeconds * 1000.0);

    public static final boolean includeDebugTabs = true;
	public static final boolean updatePIDsFromShuffleboard = true;
	public static final boolean outputTelemetry = true;

	public static class ControlBoardConstants {
		public static final double kSwerveTranslationDeadband = 0.15;
		public static final double kSwerveRotationDeadband = 0.10;
		public static final double kTriggerThreshold = 0.3;//how much a trigger must be pressed before it counts as being held down
		public static final double kTranslationJoystickSmoothingExponent = 1.5;
		public static final double kRotationJoystickSmoothingExponent = 1.5;
		/**we have a trigger that, when held, slows the robot's translation down by a scalar whose scale is based on how far the trigger is held down. 
		 * This constant is the minimum value, when the trigger is held all the ways down */
		public static final double kSlowModeTranslationMinScalar = 0.20;
		public static final double kSlowModeRotationMinScalar = 0.20;

		public static final Rotation2d kThresholdToSnapSwerveToPole = Rotation2d.fromDegrees(7);
		public static final double manualPivotPODeadband = 0.1;
        public static final Rotation2d kTweakArmSetpointAmount = Rotation2d.fromDegrees(0.7);
	}

	/** 
	 *    Our system has 0,0 where the  white center line meets the wall on the feeder station side. 
	 * 	  We have forwards(gyro angle) as 90 degrees on both sides, so on blue x is always positive, and on red, x is negative. negative y is on our side of the white line, 
	 *    and positive y is on the opposing alliance's side of the whilte line. <p>
	 * 
	 *    All of the distance constants in our code are in meters.
	 */
	public static class FieldConstants {
		public static final Translation2d fieldDimensions = new Translation2d(8.0, 16.51);

		public static final double hybridLevelY = -7.0711;
		public static final double middleLevelY = -7.4679;
		public static final double topLevelY = -7.9016;
		public static final double apriltagTargetY = -7.26;

		public static final double hybridLevelZ = 0.0;
		public static final double middleLevelZCube = 0.5969;
		public static final double middleLevelZCone = 0.8636;
		public static final double topLevelZCube = 0.9017;
		public static final double topLevelZCone = 1.1684;

		public static final double stagingMarkY = -1.203;
		
		public static final MirroredTranslation stagingMark1 = new MirroredTranslation(new Translation2d(3.428-0.08, stagingMarkY), new Translation2d(0.0, 0.0), new Translation2d(0.0, 0.0));// closest game  piece  to  the  feeder station
		public static final MirroredTranslation stagingMark2 = new MirroredTranslation(new Translation2d(4.647, stagingMarkY));
		public static final MirroredTranslation stagingMark3 = new MirroredTranslation(new Translation2d(5.866+0.35, stagingMarkY+0.15), new Translation2d(), new Translation2d(0.0, 0.0));
		public static final MirroredTranslation stagingMark4 = new MirroredTranslation(new Translation2d(7.086+0.08, stagingMarkY), new Translation2d(), new Translation2d(0.0, 0.0));

		public static final MirroredTranslation chargeStationFL = new MirroredTranslation(new Translation2d(4.0375, -3.4183));
		public static final MirroredTranslation chargeStationFR = new MirroredTranslation(new Translation2d(4.0375, -5.3511));
		public static final MirroredTranslation chargeStationBL = new MirroredTranslation(new Translation2d(6.5077, -3.4183));
		public static final MirroredTranslation chargeStationBR = new MirroredTranslation(new Translation2d(6.5077, -5.3511));

		public static final MirroredTranslation chargeStationcenter = new MirroredTranslation(new Translation2d(5.35, -4.39), new Translation2d(0.0, 0.0), new Translation2d());
		public static final MirroredTranslation chargeStationNode1SideTarget = new MirroredTranslation(new Translation2d(5.35-0.45, -4.39), new Translation2d(0.0, 0.0), new Translation2d());
		public static final MirroredTranslation chargeStationNode9SideTarget = new MirroredTranslation(new Translation2d(5.35+0.45, -4.39), new Translation2d(0.0, 0.0), new Translation2d());

		public static final MirroredTranslation cableBump = new MirroredTranslation(7.27, -4.38);

		public static final double node1X = 3.0331;// starts on the closest to the feeder station
		public static final double node2X = 3.5919-0.04;
		public static final double node3X = 4.1507;
		public static final double node4X = 4.7095;
		public static final double node5X = 5.2683;
		public static final double node6X = 5.8271;
		public static final double node7X = 6.3684;
		public static final double node8X = 6.9447+0.10;
		public static final double node9X = 7.575;

		public static final double SwerveZeroBeforeAutoPlaceY = -6.32;
		public static final double shootY = -3.23+1;
		public static final double coneSwervePlacementY = -6.5657;
		public static final double cubeSwervePlacementY = -6.5657;
		
		public static final double coneSwervePlacementYAuto = coneSwervePlacementY;
		public static final double cubeSwervePlacementYAuto = cubeSwervePlacementY + 0.25;

		static MirroredTranslation[] visionTargetLocations = new MirroredTranslation[]{
			new MirroredTranslation(new Translation2d(node1X, middleLevelY)),
			new MirroredTranslation(new Translation2d(node2X, apriltagTargetY)),
			new MirroredTranslation(new Translation2d(node3X, middleLevelY)),
	
			new MirroredTranslation(new Translation2d(node4X, middleLevelY)),
			new MirroredTranslation(new Translation2d(node5X, apriltagTargetY)),
			new MirroredTranslation(new Translation2d(node6X, middleLevelY)),
	
			new MirroredTranslation(new Translation2d(node7X, middleLevelY)),
			new MirroredTranslation(new Translation2d(node8X, apriltagTargetY)),
			new MirroredTranslation(new Translation2d(node9X, middleLevelY))			
		};
		
		static MirroredTranslation[][] nodeLocations = new MirroredTranslation[][]{
			{new MirroredTranslation(node1X, hybridLevelY), new MirroredTranslation(node1X, middleLevelY), new MirroredTranslation(node1X, topLevelY)},
			{new MirroredTranslation(node2X, hybridLevelY), new MirroredTranslation(node2X, middleLevelY), new MirroredTranslation(node2X, topLevelY)},
			{new MirroredTranslation(node3X, hybridLevelY), new MirroredTranslation(node3X, middleLevelY), new MirroredTranslation(node3X, topLevelY)},
			{new MirroredTranslation(node4X, hybridLevelY), new MirroredTranslation(node4X, middleLevelY), new MirroredTranslation(node4X, topLevelY)},
			{new MirroredTranslation(node5X, hybridLevelY), new MirroredTranslation(node5X, middleLevelY), new MirroredTranslation(node5X, topLevelY)},
			{new MirroredTranslation(node6X, hybridLevelY), new MirroredTranslation(node6X, middleLevelY), new MirroredTranslation(node6X, topLevelY)},
			{new MirroredTranslation(node7X, hybridLevelY), new MirroredTranslation(node7X, middleLevelY), new MirroredTranslation(node7X, topLevelY)},
			{new MirroredTranslation(node8X, hybridLevelY), new MirroredTranslation(node8X, middleLevelY), new MirroredTranslation(node8X, topLevelY)},
			{new MirroredTranslation(node9X, hybridLevelY), new MirroredTranslation(node9X, middleLevelY), new MirroredTranslation(node9X, topLevelY)}
		};
	
		///////////////////////// This is where we create offsets for competition//////////////////////
		static {
			// stagingMark1.offset(new Translation2d(0.0, -0.5));
			// stagingMark2.offset(new Translation2d(-0.1, -.3));
			// stagingMark3.offset(new Translation2d(0.1, -0.3));
			// gamePiece4SwerveLocation.configOffsets(new Translation2d(), new Translation2d(-0.15, 0));
			// stagingMark4.offset(new Translation2d(0.0, -0.5));
		}

		/////////////////////////////////////////////////////////////////////////////////////////////////
	}
	
	public static class PlacementConstants{
		public static final MirroredTranslation shootLocation1 = new MirroredTranslation(FieldConstants.node1X, FieldConstants.shootY);
		public static final MirroredTranslation shootLocation2 = new MirroredTranslation(FieldConstants.node2X, FieldConstants.shootY);
		public static final MirroredTranslation shootLocation3 = new MirroredTranslation(FieldConstants.node3X, FieldConstants.shootY);
		public static final MirroredTranslation shootLocation4 = new MirroredTranslation(FieldConstants.node4X, FieldConstants.shootY);
		public static final MirroredTranslation shootLocation5 = new MirroredTranslation(FieldConstants.node5X, FieldConstants.shootY);
		public static final MirroredTranslation shootLocation6 = new MirroredTranslation(FieldConstants.node6X, FieldConstants.shootY);
		public static final MirroredTranslation shootLocation7 = new MirroredTranslation(FieldConstants.node7X, FieldConstants.shootY);
		public static final MirroredTranslation shootLocation8 = new MirroredTranslation(FieldConstants.node8X, FieldConstants.shootY);
		public static final MirroredTranslation shootLocation9 = new MirroredTranslation(FieldConstants.node9X, FieldConstants.shootY);

		public static final double distanceForCubeAutoIntake = 0.32;

		public static final MirroredTranslation gamePiece1SwerveLocation = FieldConstants.stagingMark1.plus(new Translation2d(distanceForCubeAutoIntake, Rotation2d.fromDegrees(-90)));
		public static final MirroredTranslation gamePiece2SwerveLocation = FieldConstants.stagingMark2.plus(new Translation2d(distanceForCubeAutoIntake, Rotation2d.fromDegrees(-130)));
		public static final MirroredTranslation gamePiece3SwerveLocation = FieldConstants.stagingMark3.plus(new Translation2d(distanceForCubeAutoIntake-0.35, Rotation2d.fromDegrees(-35)));
		public static final MirroredTranslation gamePiece4SwerveLocation = FieldConstants.stagingMark4.plus(new Translation2d(distanceForCubeAutoIntake-0.15, Rotation2d.fromDegrees(-90)));

		public static final MirroredTranslation gamePiece2ShootPathLocation1 = FieldConstants.stagingMark2.plus(new Translation2d(distanceForCubeAutoIntake, Rotation2d.fromDegrees(-115)));
		public static final MirroredTranslation gamePiece3ShootPathLocation1 = FieldConstants.stagingMark3.plus(new Translation2d(distanceForCubeAutoIntake, Rotation2d.fromDegrees(-115)).plus(new Translation2d(0.0, 0.0)));
		public static final MirroredTranslation gamePiece4ShootPathLocation1 = FieldConstants.stagingMark4.plus(new Translation2d(distanceForCubeAutoIntake, Rotation2d.fromDegrees(-115)).plus(new Translation2d(0.0, 0.0)));

		public static final MirroredTranslation gamePiece2ShootPathLocation9 = FieldConstants.stagingMark2.plus(new Translation2d(distanceForCubeAutoIntake, Rotation2d.fromDegrees(-70)).plus(new Translation2d(0.3, 0)));
		public static final MirroredTranslation gamePiece3ShootPathLocation9 = FieldConstants.stagingMark3.plus(new Translation2d(distanceForCubeAutoIntake, Rotation2d.fromDegrees(-70)));
		public static final MirroredTranslation gamePiece4ShootPathLocation9 = FieldConstants.stagingMark4.plus(new Translation2d(distanceForCubeAutoIntake, Rotation2d.fromDegrees(-70)));
		
		public static final MirroredPose singleSubstationConeRetrievalPoint = new MirroredPose(0.35, 5.575, Rotation2d.fromDegrees(180));
		public static final MirroredTranslation swerveZeroBeforeSubstationPoint = new MirroredTranslation(0.8, 5.575);

		static MirroredPose[] swervePlacementStates = new MirroredPose[]{
			new MirroredPose(FieldConstants.node1X, FieldConstants.coneSwervePlacementY, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node2X, FieldConstants.cubeSwervePlacementY, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node3X, FieldConstants.coneSwervePlacementY, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node4X, FieldConstants.coneSwervePlacementY, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node5X, FieldConstants.cubeSwervePlacementY, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node6X, FieldConstants.coneSwervePlacementY, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node7X, FieldConstants.coneSwervePlacementY, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node8X, FieldConstants.cubeSwervePlacementY, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node9X, FieldConstants.coneSwervePlacementY, SwerveConstants.robotForwardAngle)
		};

		static MirroredPose[] swerveAutoPlacementStates = new MirroredPose[]{
			new MirroredPose(FieldConstants.node1X, FieldConstants.coneSwervePlacementYAuto, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node2X, FieldConstants.cubeSwervePlacementYAuto, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node3X, FieldConstants.coneSwervePlacementYAuto, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node4X, FieldConstants.coneSwervePlacementYAuto, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node5X, FieldConstants.cubeSwervePlacementYAuto, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node6X, FieldConstants.coneSwervePlacementYAuto, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node7X, FieldConstants.coneSwervePlacementYAuto, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node8X, FieldConstants.cubeSwervePlacementYAuto, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node9X, FieldConstants.coneSwervePlacementYAuto, SwerveConstants.robotForwardAngle)
		};

		static MirroredPose[] swerveBackupBeforePlaceStates = new MirroredPose[]{
			new MirroredPose(FieldConstants.node1X, FieldConstants.SwerveZeroBeforeAutoPlaceY, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node2X, FieldConstants.SwerveZeroBeforeAutoPlaceY, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node3X, FieldConstants.SwerveZeroBeforeAutoPlaceY, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node4X, FieldConstants.SwerveZeroBeforeAutoPlaceY, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node5X, FieldConstants.SwerveZeroBeforeAutoPlaceY, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node6X, FieldConstants.SwerveZeroBeforeAutoPlaceY, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node7X, FieldConstants.SwerveZeroBeforeAutoPlaceY, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node8X, FieldConstants.SwerveZeroBeforeAutoPlaceY, SwerveConstants.robotForwardAngle),
			new MirroredPose(FieldConstants.node9X, FieldConstants.SwerveZeroBeforeAutoPlaceY, SwerveConstants.robotForwardAngle)
		};

	}

	public static class VisionConstants {


		public static class FrontLimelightConstants{
			public static final Matrix<N3, N1> retroReflectiveMeasurementStandardDeviations = VecBuilder.fill(0.0004, 0.0004, 0.05);// x, y, theta... We aren't using theta in the vision measurements, so put really high stddevs for it as a workaround
			public static final int kAprilTagPipeline = 0;
			public static final int kConeLeftPipeline = 1;
			public static final int kConeRightPipeline = 2;

			//the highest TA values for the limelight; These were the measured TA values with the limelight as close to the targets as possible.
			public static final double maxApriltagTA = 2.146;
			public static final double maxRetroReflectiveTA = 0.694;


			//Limelight mounting constants, offsets from center of robot
			public static final double kForwardOffsetMeters = 0.3048;
			public static final double kRightOffsetMeters = 0.2286;

			
			//Constants for filtering out possibly bad limelight measurements
			public static final Rotation2d angleThresholdToCountRetroReflectiveMeasurement = Rotation2d.fromDegrees(5);
			public static final double kMaxSpeedForVisionUpdateTeleop = 3.5;
			public static final double kMaxSpeedForVisionUpdateAuto = 0.45;
			public static final double swervePoseErrorToDiscardApriltagMeasurement = 3.0;

				
			//These are the maps we use to get vision measurements from the retroreflective vision tape. We put the robot in its ideal placement location and measure the tx and ty values. At this point,
			//the offset from the placement state is (0, 0). We then change the x and y position of the robot and measure how far off the robot its(using the odometry, because it is easier to use than a tape measure).
			//These maps hold all of the offset values, so we can measure how far the robot is off by looking up the offset from the limelight's given tx and ty values
			public static final InterpolatingTreeMap<Double, Double> distanceFromConeVisionTargetYMap = new InterpolatingTreeMap<Double, Double>();
			/** robotCentric x offsets. Robot will move right if x is positive, regardless of driverstations		 */
			public static final InterpolatingTreeMap<Double, Double> distanceFromConeVisionTargetXMap = new InterpolatingTreeMap<Double, Double>();
			static {
				distanceFromConeVisionTargetXMap.put(-30.00, 0.206108345037028);
				distanceFromConeVisionTargetXMap.put(-27.19, 0.148211326040494);
				distanceFromConeVisionTargetXMap.put(-18.06, 0.0);
				distanceFromConeVisionTargetXMap.put(-9.84, -0.10620859873514);
				distanceFromConeVisionTargetXMap.put(-1.6, -0.203501697162219);
				distanceFromConeVisionTargetXMap.put(12.9, -0.390186872789435);
				distanceFromConeVisionTargetXMap.put(20.73, -0.498079751654977);
				distanceFromConeVisionTargetXMap.put(31.21, -0.658726720972104);

				distanceFromConeVisionTargetYMap.put(-15.83, 0.00);
				distanceFromConeVisionTargetYMap.put(-14.66, 0.097252418838486);
				distanceFromConeVisionTargetYMap.put(-13.17, 0.235899708266877);
				distanceFromConeVisionTargetYMap.put(-12.13, 0.359712206170641);
				distanceFromConeVisionTargetYMap.put(-11.35, 0.504180902428665);
				distanceFromConeVisionTargetYMap.put(-10.58, 0.64266281057651);
				distanceFromConeVisionTargetYMap.put(-10.25, 0.736428348453972);
			}
		
			//TreeMaps for TA to standard deviations. The lower the TA value, the less confident we are in our measurements, so the higher the standard deviations must be.
			public static final InterpolatingTreeMap<Double, Double> aprilTagTAToTranslationSTDDevs = new InterpolatingTreeMap<Double, Double>();
			public static final InterpolatingTreeMap<Double, Double> aprilTagTAToAngleSTDDevs = new InterpolatingTreeMap<Double, Double>();


			public static final InterpolatingTreeMap<Double, Double> retroReflectiveTAToTranslationSTDDevScalarMap = new InterpolatingTreeMap<Double, Double>();
			public static final InterpolatingTreeMap<Double, Double> retroReflectiveTAToAngleSTDDevScalarMap = new InterpolatingTreeMap<Double, Double>();

			static{
				aprilTagTAToTranslationSTDDevs.put(maxApriltagTA, 0.05);//TODO figure out how to actually measure stdDevs. These are mostly guesses to make it work.
				aprilTagTAToTranslationSTDDevs.put(1.0, 0.1);
				aprilTagTAToTranslationSTDDevs.put(0.1, 1.0);
				aprilTagTAToTranslationSTDDevs.put(0.05, 5.0);
				aprilTagTAToTranslationSTDDevs.put(0.0, Double.MAX_VALUE);

				aprilTagTAToAngleSTDDevs.put(maxApriltagTA, 0.01);


				retroReflectiveTAToTranslationSTDDevScalarMap.put(maxRetroReflectiveTA, 1.0);//we default the scaling to be 1 for now
				retroReflectiveTAToAngleSTDDevScalarMap.put(maxRetroReflectiveTA, 1.0);
			}
		}

		public static class BackLimelightConstants{
			public static final int kSubstationTagPipeline = 7;
			public static final int kRobotBootedUpPipeline = 9;

			public static final Matrix<N3, N1> substationTagMeasurementStandardDeviations = VecBuilder.fill(0.0004, 0.0004, 0.05);


			public static final double kMaxAprilTagTA = 2.027;//TODO maybe remove since we don't use apriltags with back limelight anymore

			//Constants for filtering out possibly bad limelight measurements
			public static final Rotation2d angleThresholdToCountSubstationTagMeasurement = Rotation2d.fromDegrees(5);
			public static final double kMaxSpeedForVisionUpdateTeleop = 3.5;
			public static final double kMaxSpeedForVisionUpdateAuto = 0.3;

			public static final InterpolatingTreeMap<Double, Double> aprilTagTAToSTDDevs = new InterpolatingTreeMap<Double, Double>();//TODO maybe remove since we don't use apriltags with back limelight anymore
			static{
				aprilTagTAToSTDDevs.put(kMaxAprilTagTA, 0.05);
				aprilTagTAToSTDDevs.put(1.0, 0.1);
				aprilTagTAToSTDDevs.put(0.1, 1.0);
				aprilTagTAToSTDDevs.put(0.05, 5.0);
				aprilTagTAToSTDDevs.put(0.0, Double.MAX_VALUE);
			}
			
			public static final InterpolatingTreeMap<Double, Double> txToMetersSubstationTagMap = new InterpolatingTreeMap<Double, Double>();
			public static final InterpolatingTreeMap<Double, Double> tyToMetersSubstationTagMap = new InterpolatingTreeMap<Double, Double>();
			
			static{
				txToMetersSubstationTagMap.put(25.66, -2.007);
				txToMetersSubstationTagMap.put(18.68, -0.063);
				txToMetersSubstationTagMap.put(13.5, 0.0);
				txToMetersSubstationTagMap.put(5.62, 0.135);
				txToMetersSubstationTagMap.put(-4.86, 0.344);
				txToMetersSubstationTagMap.put(-16.76, 0.545);
				txToMetersSubstationTagMap.put(-25.35, 0.705);

				tyToMetersSubstationTagMap.put(15.10, 0.0);
				tyToMetersSubstationTagMap.put(7.07, 0.262);
				tyToMetersSubstationTagMap.put(2.9, 0.452);
				tyToMetersSubstationTagMap.put(-0.66, 0.667);
				tyToMetersSubstationTagMap.put(-4.30, 0.953);
				tyToMetersSubstationTagMap.put(-7.56, 1.317);
				tyToMetersSubstationTagMap.put(-10.84, 1.846);
				tyToMetersSubstationTagMap.put(-12.67, 2.300);
				tyToMetersSubstationTagMap.put(-14.36, 2.881);
				tyToMetersSubstationTagMap.put(-15.93, 3.571);

			}
		}

		public static final double kLimelightLatency = 0.02;// TODO measure
        public static final double limelightSwitchPipelineDelay = 0.014;
	}
	
	public static class ArmConstants {
		/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
		public static final double kPivotTicksPerRotation = 2048.0;
		public static final double kTelescopeTicksPerRotation = 2048.0;

		public static final double kPivotGearRatio = 4.0 * 3.0 * 3.0 * 60.0 / 22.0;
		public static final double kTelescopeGearRatio = 100.0 / 1.0;
		public static final Rotation2d PivotAngleOffset = Rotation2d.fromDegrees(119.057552083);// adding makes it more negative

		public static final double kMinTelescopeLength = 0.47;//meters
		public static final double kMaxTelescopeLength = 1.138;//meters  //measured from pivot center to right past the cap that holds the end effector on the end of the telescope, does not include end effector
		public static final double kPotentiometerMin = 0;//potentiometer reading at minimum arm extension
		public static final double kPotentiometerMax = 7135;//potentiometer reading at maximum arm extension
		public static final double kDistanceForSoftLimit = 150;// native potentiometer units

		public static final double kPivotCruiseVelocity = 21742 / 0.7;// measured 21742.0
		public static final MotionMagicConstants pivotMotionMagicConstants = new MotionMagicConstants(
				kPivotCruiseVelocity, kPivotCruiseVelocity / 0.7, 0);

		public static final double kTelescopeCruiseVelocity = 4000;
		public static final double kTelescopeAcceleration = 5000;
		public static final MotionMagicConstants telescopeMotionMagicConstants = new MotionMagicConstants(
				kTelescopeCruiseVelocity, kTelescopeAcceleration, 1);

		public static final PIDConstants pivotPIDConstants = new PIDConstants(0.15, 0.000, 0);
		static {
			pivotPIDConstants.setIntegralZone(2000);
		}
		public static final PIDConstants telescopePIDConstants = new PIDConstants(2.2, 0.0, 0);
		static {
			telescopePIDConstants.setIntegralZone(700);
		}
		public static final Rotation2d kPivotAngleOnTargetThreshold = Rotation2d.fromDegrees(3.0);
		public static final double kTelescopeAtTargetThreshold = 0.05;// meters
		public static final double kSuckInLength = kMinTelescopeLength + .01;	


		/** The angle differnce of the arm as it wiggles without moving the chain. When measuring setpoints for the arm, we always measure the location of the arm. When setting the arm to
		 * a setpoint, we add half of the slop in the direction opposite of gravity, so that when the arm drops lower because of play in the system, it drops to the correct position. */
		public static final Rotation2d kArmSlop = Rotation2d.fromDegrees(4.4);

		public static final Rotation2d kPivotAngleThresholdForTelescopeOut = Rotation2d.fromDegrees(75.0);

		public static final double pivotForwardSoftLimit = Double.MAX_VALUE;
		public static final double pivotReverseSoftLimit = -Double.MAX_VALUE;//we never ended up setting a soft limit.

		public static double telescopeKSWithoutCone = 0.408;// output to overcome friction //should be final, but can't be for tuning
		public static double telescopeKFGravityWithoutCone = 0.256;// output to overcomeGravity(mutliplied by sin(theta))
		public static double telescopeKSWithCone = 0.53488;// output to overcome friction //should be final, but can't be for tuning
		public static double telescopeKFGravityWithCone = 0.3651;// output to overcomeGravity(mutliplied by sin(theta))

		//Maps for the feedforward. The input is the telescope length, the output is the percentoutput for an ArbitraryFeedForward
		public static final InterpolatingTreeMap<Double, Double> gravityFeedforwardMapWithoutCone = new InterpolatingTreeMap<Double, Double>();
		public static final InterpolatingTreeMap<Double, Double> gravityFeedforwardMapWithCone = new InterpolatingTreeMap<Double, Double>();
		static {
			gravityFeedforwardMapWithoutCone.put(kMinTelescopeLength, 0.035 * 18 / 22);
			gravityFeedforwardMapWithoutCone.put(kMaxTelescopeLength, .072 * 18 / 22);

			gravityFeedforwardMapWithCone.put(kMinTelescopeLength, .072 * 18 / 22);
			gravityFeedforwardMapWithCone.put(kMaxTelescopeLength, .095 * 18 / 22);
		}
		
		public static class Setpoints{
			// All of the setpoints for the arm should be measured using the actual arm position so that if the wiggle room changes, we only need to change that constants
			public static final Translation2d kConeRetrieval = new Translation2d(kMinTelescopeLength, Rotation2d.fromDegrees(-102.9));
			public static final Translation2d kConeHold = new Translation2d(kMinTelescopeLength, Rotation2d.fromDegrees(-88.0));

			public static final Translation2d kAutoStart = new Translation2d(0.5, Rotation2d.fromDegrees(-43.5));
			public static final Translation2d kPreparePlacement = kAutoStart;
			public static final Translation2d kCubeRetrieval = new Translation2d(kMinTelescopeLength, Rotation2d.fromDegrees(-105));
			public static final Translation2d kPoopShoot = kAutoStart;
			
			//arm setpoint for when we have/don't have a game piece in auto. When our arm is inside the robot at rest position, the claw still sticks slightly out of frame perimeter.
				//This means that we cannot stick our intake out for more than a 'momentary' amount of time or we will get penalties. In auto, we grab and score the cubes fast enough that
				//it generally doesn't matter, but if we need to change it, we only need to change one of these setpoints. In teleop, we have to bring the arm up while grabbing a cube
			public static final Translation2d kAutoWithGamePiece = kCubeRetrieval;
            public static final Translation2d kAutoWithoutGamepiece = kCubeRetrieval;

			public static final Translation2d kTopCone = new Translation2d(1.10712, Rotation2d.fromDegrees(64.2));
			public static final Translation2d kMiddleCone = new Translation2d(0.60, Rotation2d.fromDegrees(74.0));
			public static final Translation2d kHybridCone = new Translation2d(ArmConstants.kMinTelescopeLength+ 0.05, Rotation2d.fromDegrees(122));

			public static final Translation2d kTopConeAutoStart = new Translation2d(1.05, Rotation2d.fromDegrees(61.0));//we have a different setpoint for the arm placement for auto because we slam the cones down differently
			public static final Translation2d kMiddleConeAutoStart = new Translation2d(0.623, Rotation2d.fromDegrees(70));
			public static final Translation2d kHybridConeAutoStart = new Translation2d(ArmConstants.kMinTelescopeLength+ 0.05, Rotation2d.fromDegrees(122));
		}
	}

	public static class IntakeConstants {
		public static final double kWaitBeforeRunWheelsDuration = 0.20;
		public static final double kIntakePO = 0.9;
		public static final double kEjectPO = -0.6;
		public static final double kShootHighPO = 0.68;
		public static final double kShootMidPO = 0.45;
		public static final double kShootHighAutoPO = 0.74;
		public static final double kShootMidAutoPO = 0.42;
		public static final double kRunWhileRetractedDuration = 0.15;
		public static final double kSweepPO = -0.3;
		public static final double kAutoIntakePO = 0.7;
		public static final double kRetractAndRunPO = 0.0;

		public static class ConveyorConstants {
			public static final double timeAfterBeamBreakToStop = 0.25;
			public static final double timeAfterBeamBreakToStartAgain = 0.8;
		}

		public static class UpperConveyorConstants {
			public static final double kIntakePO = 0.5;
			public static final double kEjectPO = -0.5;
			public static final double kShootHighPO = -0.63;
			public static final double kShootMidPO = -0.40;
			public static final double kShootHighAutoPO = -0.68;
			public static final double kShootMidAutoPO = -0.40;
			public static final double kRetractAndRunPO = 0.7;
			public static final double kBackwardsEjectPO = 0.0;
            public static final double kAutoIntakePO = 0.4;
		}

		public static class LowerConveyorConstants {
			public static final double kIntakePO = .5;
			public static final double kEjectPO = -.5;
			public static final double kShootHighPO = -.70;
			public static final double kShootMidPO = -.6;
			public static final double kShootHighAutoPO = -.7;
			public static final double kShootMidAutoPO = -.6;

			public static final double kRetractAndRunPO = 0.7;
			public static final double kBackwardsEjectPO = 1;

            public static final double kAutoIntakePO = 0.2;
            public static final double kPreparePoopShoot = -0.3;
			public static final double kPoopShootFromChargeLinePO = 1.0;
            public static final double kPoopShootFromChargeLineAutoPO = 1.0;
            public static final double kPrepareForShot = -1.0;
            public static final double kEjectOnlyLowerConveyorPO = -0.7;
		}

		public static class ShooterConstants {
			public static final double kBackwardsEjectPO = 0.3;
            public static final double kTimeBeforeCanShoot = 0.18;
            public static final double kPoopShootFromChargeLinePO = 0.42;
            public static final double kPoopShootFromChargeLineAutoPO = 0.38;
			public static final double kDefaultAutoShootDuration = 0.85;
		}

		public static class RodConstants{
			public static final int startSensorPosition = 0+354;
			public static final int verticalSensorPosition = -354+354;
			public static final int horizonalSensorPosition = -1360+354;

			public static final int sensorDistanceFromSoftLimit = 75;
			public static final int targetOutPosition = horizonalSensorPosition+sensorDistanceFromSoftLimit;
			public static final int targetInPosition = startSensorPosition-sensorDistanceFromSoftLimit;

			public static final double gearRatio =  90.0/horizonalSensorPosition;

			public static final PIDConstants kRodPID = new PIDConstants(6, 0, 0);
			public static final MotionMagicConstants motionMagicConstants = new MotionMagicConstants(500, 400, 0);
			public static final double kRodGravityFeedforward = 0.02;
		}
	}


	/** This record holds the settings for an auto. We have a slow, medium, and fast version of this class and we select which config we want when referencing a trajectory */
	public static class SwerveAutoConfig {
		public double kAutoMaxSpeed; // m/s
		public double kAutoMaxAcceleration; // m/s^2
		public double kAutoMaxDeceleration; // rad/s^2

		public SwerveAutoConfig(double speed, double acceleration, double deceleration) {
			kAutoMaxSpeed = speed;
			kAutoMaxAcceleration = acceleration;
			kAutoMaxDeceleration = deceleration;
		}
	}

	public static class SwerveConstants {
		/** this is the robot angle for forward, the side we retrive cones from and shoot from the poop shooter */
		public static final Rotation2d robotForwardAngle = Rotation2d.fromDegrees(90);//all of these only work as rotation2ds because their cosine value is 0. If they were to change, they would need to be MirroredRotations
		/** this is the robot angle for backward, the side we intake/shoot cubes and place cones with the arm */
		public static final Rotation2d robotBackwardAngle = Rotation2d.fromDegrees(-90);
	
		// Robot
		public static final double kRobotWidth = 0.8128;
		public static final double kRobotLength = 0.889;
		public static final double kRobotDiagonal = Math.hypot(kRobotWidth, kRobotLength);
		public static final double kHalfRobotWidth = kRobotWidth / 2.0;
		public static final double kHalfRobotLength = kRobotLength / 2.0;
		public static final Translation2d[] bumperVertices = new Translation2d[] { // in counterclockwise order to
																					// comply with SwerveDriveHelper
				new Translation2d(kHalfRobotWidth, kHalfRobotLength),
				new Translation2d(-kHalfRobotWidth, kHalfRobotLength),
				new Translation2d(-kHalfRobotWidth, -kHalfRobotLength),
				new Translation2d(kHalfRobotWidth, -kHalfRobotLength),
		};

		// Dodging
		/**This is for our dodging moves, where the robot drives around a pivot point, making a circular path. When the driver starts holding down the dodging move button,
		 * 	 the code stores a pivot point on the field equal to this constant's distance in the direction that the driver is holding the translation joystick down. The larger this
		 * 	 constant is, the greater the radius of the dodge move's arc. */
		public static final double kDodgeArcLength = 0.75;

		// Drivebase
		public static final double kWheelBaseWidth = 0.50165; // x axis
		public static final double kWheelBaseLength = 0.57531; // y axis
		public static final double kHalfWheelBaseWidth = kWheelBaseWidth / 2.0;
		public static final double kHalfWheelBaseLength = kWheelBaseLength / 2.0;
		public static final double kRobotCenterToWheelCenter = Math.hypot(kHalfWheelBaseLength, kHalfWheelBaseWidth);
		// Motion constraints
		public static final double kMaxSpeed = 5.7349; // m/s //theoretical calculation. Equal to the module max
														// velocity * kDriveVelocityCoefficient. 22000 / 2048 / 6.12 *
														// 10 * 2*pi*0.052
		public static final double kMaxDriveSpeed = kMaxSpeed;//TODO was limited to 5.0;
		public static final double kMaxDriveAngularSpeed = 8.0;

		public static final double kMinSpeed = kMaxSpeed * 0.0005; // m/s
		public static final double kMaxAcceleration = 5.7349; // m/s^2

		public static final double kMaxAngularSpeed = 6;// kMaxSpeed / kRobotCenterToWheelCenter; // rad/s
		public static final double kMinAngularSpeed = kMaxAngularSpeed * 0.005; // rad/s

		public static final double kMaxAngularAcceleration = kMaxAcceleration / kRobotCenterToWheelCenter; // rad/s^2

		public static final Translation2d flLocation = new Translation2d(kHalfWheelBaseWidth, kHalfWheelBaseLength);
		public static final Translation2d blLocation = new Translation2d(-kHalfWheelBaseWidth, kHalfWheelBaseLength);
		public static final Translation2d brLocation = new Translation2d(-kHalfWheelBaseWidth, -kHalfWheelBaseLength);
		public static final Translation2d frLocation = new Translation2d(kHalfWheelBaseWidth, -kHalfWheelBaseLength);
		
		public static final SwerveModuleConstants FLConstants = new SwerveModuleConstants(
				SelectedHotswapModules.selectedFLModule,
				ModuleLocation.FL);
		public static final SwerveModuleConstants BLConstants = new SwerveModuleConstants(
				SelectedHotswapModules.selectedBLModule,
				ModuleLocation.BL);
		public static final SwerveModuleConstants BRConstants = new SwerveModuleConstants(
				SelectedHotswapModules.selectedBRModule,
				ModuleLocation.BR);
		public static final SwerveModuleConstants FRConstants = new SwerveModuleConstants(
				SelectedHotswapModules.selectedFRModule,
				ModuleLocation.FR);

		public static final Translation2d[] moduleLocations = new Translation2d[] {
				FLConstants.translation,
				BLConstants.translation,
				BRConstants.translation,
				FRConstants.translation
		};

		// Pose Estimator
		public static final Matrix<N3, N1> stateStandardDeviations = VecBuilder.fill(0.03, 0.03, 0.01); // x, y, theta

		// Autonomous Motion Constraints
		public static final SwerveAutoConfig fastSpeedConfig = new SwerveAutoConfig(5.0, 3.4, -2.5);
		public static final SwerveAutoConfig mediumSpeedConfig = new SwerveAutoConfig(4.0, 2.5, -1.8);
		public static final SwerveAutoConfig slowSpeedConfig = new SwerveAutoConfig(2.5, 2.0, -1.40);

		public static final TrajectoryConstraint fastSpeedConstraint = ScreamUtil.createTrajectoryConstraint(SwerveConstants.fastSpeedConfig);
		public static final TrajectoryConstraint mediumSpeedConstraint = ScreamUtil.createTrajectoryConstraint(SwerveConstants.mediumSpeedConfig);
		public static final TrajectoryConstraint slowSpeedConstraint = ScreamUtil.createTrajectoryConstraint(SwerveConstants.slowSpeedConfig);

		public static final TrajectoryConfig defaultFastSpeedConfig = new TrajectoryConfig(
				SwerveConstants.fastSpeedConfig.kAutoMaxSpeed, SwerveConstants.fastSpeedConfig.kAutoMaxAcceleration)
				.addConstraint(fastSpeedConstraint);

		public static final TrajectoryConfig defaultMediumSpeedConfig = new TrajectoryConfig(
				SwerveConstants.mediumSpeedConfig.kAutoMaxSpeed, SwerveConstants.mediumSpeedConfig.kAutoMaxAcceleration)
				.addConstraint(mediumSpeedConstraint);

		public static final TrajectoryConfig defaultSlowSpeedConfig = new TrajectoryConfig(
				SwerveConstants.slowSpeedConfig.kAutoMaxSpeed, SwerveConstants.slowSpeedConfig.kAutoMaxAcceleration)
				.addConstraint(slowSpeedConstraint);

		public static final double kAutoSlowdownForCableBumpRadius = 0.5;// meters

		public static final TrajectoryConstraint nearCableBumpConstraint = new TrajectoryConstraint() {

			@Override
			public double getMaxVelocityMetersPerSecond(Pose2d poseMeters, double curvatureRadPerMeter,
					double velocityMetersPerSecond) {
				return 1.5;
			}

			@Override
			public MinMax getMinMaxAccelerationMetersPerSecondSq(Pose2d poseMeters, double curvatureRadPerMeter,
					double velocityMetersPerSecond) {
				return new MinMax(-1.5, 1.75);
			}
		};

		// Auto balance
		public static PIDConstants autoBalancePIDConstants = new PIDConstants(0.14, 0.0, 0.0);

		// Rotation Helper
		public static final double kRotationOpenLoopDuration = 0.2; // s
		public static final PIDConstants snapRotationPIDConstants = new PIDConstants(5.0, 0.0, 0.0);
		public static final PIDConstants holdRotationPIDConstants = new PIDConstants(0.5, 0, 0.0);
		static{
			snapRotationPIDConstants.setIntegralZone(0.2);
			holdRotationPIDConstants.setIntegralZone(0.2);
		}

		/** The kinematics limits for 254's swerve setpoint generator */
		public static final KinematicLimits defaultKinematicsLimits = new KinematicLimits(kMaxSpeed, kMaxAcceleration,
				kMaxAngularSpeed);

		//PID constants for different swerve modes
		public static final PIDConstants positionXPIDConstants = new PIDConstants(2.5, 0.0, 0.0);
		public static final PIDConstants positionYPIDConstants = new PIDConstants(2.5, 0.0, 0.0);
		public static final PIDConstants positionThetaPIDConstants = snapRotationPIDConstants;//new PIDConstants(3.5, 1.5, 0);
		public static final PIDConstants trajectoryTranslationPIDConstants = new PIDConstants(0.7, 0, 0);
		public static final PIDConstants autoPlaceXPIDConstants = new PIDConstants(4.0, 0.0, 0.0);
		public static final PIDConstants autoPlaceYPIDConstants = new PIDConstants(4.5, 0.0, 0.0);
		public static final PIDConstants autoPlaceThetaPIDConstants = positionThetaPIDConstants;
		static{
			// positionThetaPIDConstants.setIntegralZone(0.2);
		}
        public static final PIDConstants alignSingleSubstationXPIDConstants = new PIDConstants(4.0, 0.0, 0.0);
        public static final PIDConstants alignSingleSubstationThetaPIDConstants = new PIDConstants(4.5, 0.0, 0.0);
        public static final PIDConstants alignSingleSubstationYPIDConstants = positionThetaPIDConstants;
	

		//Tolerances for different swerve modes
		public static final double trajectoryTranslationTolerance = 0.3;
		public static final Rotation2d trajectoryAngleTolerance = Rotation2d.fromDegrees(7.5);
		public static final double positionPreciseTranslationTolerance = 0.2;
		public static final Rotation2d positionPreciseAngleTolerance = Rotation2d.fromDegrees(6);
		public static final double positionWideTranslationTolerance = 0.32;
		public static final Rotation2d positionWideAngleTolerance = Rotation2d.fromDegrees(10);
		public static final double visionXTolerance = 0.1;
		public static final double visionYTolerance = 0.1;
		public static final Rotation2d visionThetaTolerance = positionPreciseAngleTolerance;

		// The position and angle thresholds that the swerve must reach before switching to retroreflective vision measurements for fully automatic placement */
		public static final double positionErrorForLimelightVisionTargetMode = .25;
		public static final Rotation2d angleErrorForLimelightVisionTargetMode = Rotation2d.fromDegrees(5);

        /**The minimum distance the swerve must be from a point to try to face it. If the swerve is really close to the point we tell it to face, it might spin around crazy since the angle will change really fast at small distances*/
        public static final double facePointMinimumDistance = 0.1;

        public static final double maxSpeedDuringPostionMode = 3.0;
		public static final double disabledTimeBeforeLockWheels = 0.5;
        public static final double kPitchToConsiderOnChargeStation = 13.0;
        public static final double defaultTrajectoryTimeoutSeconds = 3.0;
		public static final double kRunOntoChargeStationSpeed = 2.5;
        public static final double kDriveAfterPitchThresholdMetAutoBalance = 0.70;
		public static final double kBackUpChargeStationJammedSpeed = 3.5;
		public static final double kPitchToStopSwerveDuringBalance = 5.25;
        public static final int swerveSpeedBufferSize = 10;
	}

	public static class SwerveModuleConstants {
		// Same for all modules
		public final double kSteerTicksPerRotation = 2048.0;
		public final double kDriveTicksPerRotation = 2048.0;
		public final double kSteerGearRatio = (60.0 / 10.0) * (50.0 / 14.0);
		public final double kDriveGearRatio = 6.12;
		public final double kWheelRadius = 0.048 * 1.051 * 0.9826; // m
		public final double kWheelCircumference = kWheelRadius * 2 * Math.PI; // m

		public final double kMaxDriveVelocity = 22000;// native units, measured 20000 with L3s
		public final MotionMagicConstants steerMotionMagicConstants = new MotionMagicConstants(kMaxDriveVelocity * 0.8,
				kMaxDriveVelocity / 0.3, 1);
		public final Rotation2d kAngleOnTargetThreshold = Rotation2d.fromDegrees(0.5);
		public static final double kDriveSlewRate = 18.0;

		public enum ModuleLocation {
			FL, BL, BR, FR
		}

		// Module specific
		public final String name;
		public PIDConstants steerPIDConstants;// these should be final, but they can't be for pid tuning
		/**The percent output required for the steer motor to overcome friction */
		public double steerKS;
		public PIDConstants drivePIDConstants;
		/**The percent output required for the drive motor to overcome friction */
		public double driveKS;
		public final Rotation2d offset;
		public final Translation2d translation;
		public final CanDeviceId driveID;
		public final CanDeviceId steerId;
		public final CanDeviceId encoderID;

		public SwerveModuleConstants(HotSwapConstants hotSwapConstants, ModuleLocation moduleLocation) {
			this.name = moduleLocation.toString();
			this.steerPIDConstants = hotSwapConstants.steerPIDConstants;
			this.drivePIDConstants = hotSwapConstants.drivePIDConstants;
			this.offset = hotSwapConstants.getModuleOffset(moduleLocation);
			this.driveID = hotSwapConstants.driveID;
			this.steerId = hotSwapConstants.steerId;
			this.encoderID = hotSwapConstants.encoderID;
			this.steerKS = hotSwapConstants.steerKS;
			this.driveKS = hotSwapConstants.driveKS;
			switch (moduleLocation) {
				case BL:
					translation = SwerveConstants.blLocation;
					break;
				case BR:
					translation = SwerveConstants.brLocation;
					break;
				case FR:
					translation = SwerveConstants.frLocation;
					break;
				default:
					DriverStation.reportError("error in SwerveModuleConstants", false);
				case FL:
					translation = SwerveConstants.flLocation;
					break;
			}
		}

		public static class DefaultConfig {
			public static final PIDConstants steerConstants = new PIDConstants(0.7, 0.0, 0.0, 0);// no kF, we use an arbitrary FF in our swerve module class
			public static final double steerKS = 0.025;//percent output for the steer motor to overcome friction
			public static final PIDConstants driveConstants = new PIDConstants(0.02, 0.0, 0.0, 1023.0 / 20000);// FF calculation is 1023.0 / kMaxDriveVelocity(native units)
			public static final double driveKS = 0.015;//percent output for the drive motor to overcome friction
		}
	}

	/**Class that stores all of the relevant information for different swerve modules. Our swerve modules are custom mounted so that we can swap out them out quickly, and we
	 * use this class to store constants for all 8 of our swerve modules. <p>
	 * Because all of the the modules are 90 degrees off from each other, we can also swap out a module to any location by adding a multiple of 90 to its CANCoder's offset.
	 * All we have to do is switch the IDs for which module we are using and the code will use these constants to work with the new module
	  */
	public static class HotSwapConstants {
		public final Rotation2d offsetAtFLLocation;
		public final PIDConstants steerPIDConstants;
		public final double steerKS;
		public final PIDConstants drivePIDConstants;
		public final double driveKS;
		public final CanDeviceId driveID;
		public final CanDeviceId steerId;
		public final CanDeviceId encoderID;
		public final String busName;

		public HotSwapConstants(Rotation2d offsetAtFLLocation, PIDConstants drivePIDConstants, double driveKS,
				PIDConstants steerPIDConstants, double steerKS, ModuleIDs moduleIDs) {
					this.offsetAtFLLocation = offsetAtFLLocation;
			this.steerPIDConstants = steerPIDConstants;
			this.drivePIDConstants = drivePIDConstants;
			driveID = moduleIDs.driveID;
			steerId = moduleIDs.steerID;
			encoderID = moduleIDs.cancoderID;
			busName = moduleIDs.busName;
			this.steerKS = steerKS;
			this.driveKS = driveKS;

		}

		public HotSwapConstants(Rotation2d offsetAtFLLocation, ModuleIDs moduleIDs) {
			this(offsetAtFLLocation, DefaultConfig.driveConstants, DefaultConfig.driveKS, DefaultConfig.steerConstants,
					DefaultConfig.steerKS, moduleIDs);
		}

		public Rotation2d getModuleOffset(ModuleLocation location) {
			switch (location) {
				case BL:
					return offsetAtFLLocation.plus(Rotation2d.fromDegrees(270));
				case BR:
					return offsetAtFLLocation.plus(Rotation2d.fromDegrees(180));
				case FR:
					return offsetAtFLLocation.plus(Rotation2d.fromDegrees(90));
				default:
					DriverStation.reportError("error in HotSwapConstants", false);
				case FL:
					return offsetAtFLLocation;
			}
		}
	}

	/** List of the constants for all 8 of our swerve modules */
	public static class HotSwapModules {
		public static final HotSwapConstants moduleID1Constants = new HotSwapConstants(
				Rotation2d.fromDegrees(20.7421875-270),
				Ports.module1IDs);

		public static final HotSwapConstants moduleID2Constants = new HotSwapConstants(
				Rotation2d.fromDegrees(-141.8484375 + 180),
				Ports.module2IDs);

		public static final HotSwapConstants moduleID3Constants = new HotSwapConstants(
				Rotation2d.fromDegrees(-50.17851562499999),
				Ports.module3IDs);

		public static final HotSwapConstants moduleID4Constants = new HotSwapConstants(
				Rotation2d.fromDegrees(117.06679687499998),
				Ports.module4IDs);

		public static final HotSwapConstants moduleID5Constants = new HotSwapConstants(
				Rotation2d.fromDegrees(-167.00 + 180),
				Ports.module5IDs);

		public static final HotSwapConstants moduleID6Constants = new HotSwapConstants(
				Rotation2d.fromDegrees(179.736328125 - 90),
				Ports.module6IDs);

		public static final HotSwapConstants moduleID7Constants = new HotSwapConstants(
				Rotation2d.fromDegrees(4.39453125),
				Ports.module7IDs);

		public static final HotSwapConstants moduleID8Constants = new HotSwapConstants(
				Rotation2d.fromDegrees(-56.77734375),
				Ports.module8IDs);
	}

	public static class SelectedHotswapModules{//This is where we select which modules we are using
		public static final HotSwapConstants selectedFLModule = HotSwapModules.moduleID8Constants;
		public static final HotSwapConstants selectedBLModule = HotSwapModules.moduleID6Constants;
		public static final HotSwapConstants selectedBRModule = HotSwapModules.moduleID7Constants;
		public static final HotSwapConstants selectedFRModule = HotSwapModules.moduleID1Constants;
	}
}