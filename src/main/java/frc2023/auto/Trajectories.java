package frc2023.auto;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.constraint.EllipticalRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc2023.PlacementStates;
import frc2023.Constants.*;
import frc2023.PlacementStates.Node;
import frc2023.auto.modes.VariableAllianceTrajectory;
import frc2023.field.MirroredTranslation;
import frc2023.field.MirroredPose;
import frc2023.field.MirroredRotation;

public class Trajectories {
	
	private static Trajectories mInstance = null;
	public static Trajectories getInstance(){
		if(mInstance == null){
			mInstance = new Trajectories();
		}
		return mInstance;
	}

	private Trajectories(){}

	
	private static TrajectoryConfig createTrajectoryConfigWithCableBumpSlowdown(TrajectoryConfig config){
		return createTrajectoryConfigWithSlowdownArea(config, SwerveConstants.nearCableBumpConstraint, FieldConstants.cableBump.getPoint(DriverStation.getAlliance()), SwerveConstants.kAutoSlowdownForCableBumpRadius);
	}

	
	private static TrajectoryConfig createTrajectoryConfigWithSlowdownArea(TrajectoryConfig config, TrajectoryConstraint constraint, Translation2d point, double radius){
		return config.addConstraint(new EllipticalRegionConstraint(point, radius, radius, Rotation2d.fromDegrees(0), constraint));
	}
		

	////////////////////////////////////////////////////// Trajectories  ////////////////////////////////////////////////////////////////
	public static final Trajectory test = generateTest1();
	private static Trajectory generateTest1(){//quintic
		List<Pose2d> waypoints = Arrays.asList(
			new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(90)),
			new Pose2d(new Translation2d(0, 1), Rotation2d.fromDegrees(90))

		);
		return TrajectoryGenerator.generateTrajectory(waypoints, SwerveConstants.defaultSlowSpeedConfig);
	}

	
	public static final VariableAllianceTrajectory gamePiece1ToNode2 = new VariableAllianceTrajectory() {
		@Override
		public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {

			Pose2d start = new MirroredPose(PlacementConstants.gamePiece1SwerveLocation, new MirroredRotation(-105)).get(alliance);
			Pose2d end = new MirroredPose(PlacementStates.getSwervePlacementTranslationAuto(Node.NODE2), new MirroredRotation(-82)).get(alliance);
			List<Translation2d> waypoints = Arrays.asList(

			);
			return TrajectoryGenerator.generateTrajectory(start, waypoints, end, speedConfig);
		}
	};


	public static final VariableAllianceTrajectory node2ToGamePiece2 = new VariableAllianceTrajectory() {
		@Override
		public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
			Translation2d point1 = new MirroredTranslation(new Translation2d(3.5, -2.85)).getPoint(alliance);

			Pose2d start = new MirroredPose(PlacementStates.getSwervePlacementTranslationAuto(Node.NODE2), new MirroredRotation(105)).get(alliance);
			Pose2d end = new MirroredPose(PlacementConstants.gamePiece2SwerveLocation, new MirroredRotation(50)).get(alliance);
			List<Translation2d> waypoints = Arrays.asList(
				point1
			);
			return TrajectoryGenerator.generateTrajectory(start, waypoints, end, speedConfig);
		}
	};


	public static final VariableAllianceTrajectory gamePiece4ToNode8 = new VariableAllianceTrajectory() {
		@Override
		public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
			Translation2d point1 =  new MirroredTranslation(new Translation2d(7.30, -3.31)).getPoint(alliance);
			Translation2d point2 =  new MirroredTranslation(new Translation2d(7.30, -5.7)).getPoint(alliance);
			
			Pose2d start = new MirroredPose(PlacementConstants.gamePiece4SwerveLocation, new MirroredRotation(-96)).get(alliance);
			
			Pose2d end = new MirroredPose(PlacementStates.getSwervePlacementTranslationAuto(Node.NODE8), new MirroredRotation(-88)).get(alliance);
			List<Translation2d> waypoints = Arrays.asList(
				point1,
				point2
			);
			return TrajectoryGenerator.generateTrajectory(start, waypoints, end, createTrajectoryConfigWithCableBumpSlowdown(speedConfig));
		}
	};


	public static final VariableAllianceTrajectory node8ToGamePiece3 = new VariableAllianceTrajectory() {
		@Override
		public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
		Translation2d point1 = new MirroredTranslation(new Translation2d(7.20, -2.9)).getPoint(alliance);
		Pose2d start = new MirroredPose(PlacementStates.getSwervePlacementTranslationAuto(Node.NODE8), new MirroredRotation(75)).get(alliance);
		Pose2d end = new MirroredPose(PlacementConstants.gamePiece3SwerveLocation, new MirroredRotation(130)).get(alliance);

		List<Translation2d> waypoints = Arrays.asList(
			point1
			);

		return TrajectoryGenerator.generateTrajectory(start, waypoints, end, createTrajectoryConfigWithCableBumpSlowdown(speedConfig));
		}
	};


	public static final VariableAllianceTrajectory gamePiece2ToNode2 = new VariableAllianceTrajectory() {
		@Override
		public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
			Pose2d start = new MirroredPose(PlacementConstants.gamePiece2SwerveLocation, new MirroredRotation(-148)).get(alliance);
			Translation2d point1 = new MirroredTranslation(new Translation2d(3.55, -5.70)).getPoint(alliance);
			Pose2d end = new MirroredPose(PlacementStates.getSwervePlacementTranslationAuto(Node.NODE2), new MirroredRotation(-90)).get(alliance);

			List<Translation2d> waypoints = Arrays.asList(
				point1
			);
			return TrajectoryGenerator.generateTrajectory(start, waypoints, end, speedConfig);
		}
	};


	public static final VariableAllianceTrajectory node1ToGamePiece1 = new VariableAllianceTrajectory() {
		@Override
		public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
			Pose2d start = new MirroredPose(PlacementStates.getSwervePlacementTranslation(Node.NODE1), new MirroredRotation(90)).get(alliance);
			Translation2d point1 = new MirroredTranslation(3.15, -4.72).getPoint(alliance);
			Pose2d end = new MirroredPose(PlacementConstants.gamePiece1SwerveLocation, new MirroredRotation(90)).get(alliance);

			List<Translation2d> waypoints = Arrays.asList(
				point1
			);

			return TrajectoryGenerator.generateTrajectory(start, waypoints, end, speedConfig);
		}
	}; 


	public static final VariableAllianceTrajectory node9ToGamePiece4 = new VariableAllianceTrajectory() {
		@Override
		public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
			Pose2d start = new MirroredPose(PlacementStates.getSwervePlacementTranslation(Node.NODE9), new MirroredRotation(90)).get(alliance);
			Pose2d end = new MirroredPose(PlacementConstants.gamePiece4SwerveLocation, new MirroredRotation(90)).get(alliance);
			
			List<Pose2d> waypoints = Arrays.asList(
				start,
				end			
			);

			return TrajectoryGenerator.generateTrajectory(waypoints, createTrajectoryConfigWithCableBumpSlowdown(speedConfig));
		}
	}; 


	public static final VariableAllianceTrajectory gamePiece3ToNode8 = new VariableAllianceTrajectory() {
		@Override
		public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {

			Pose2d start = new MirroredPose(PlacementConstants.gamePiece3SwerveLocation, new MirroredRotation(-32)).get(alliance);
			// Translation2d point1 = new MirroredTranslation(7.13, -2.97).getPoint(alliance);//TODO if the path runs into charge station, uncomment this to fix it --commented 4/17/2023
			Translation2d point2 = new MirroredTranslation(7.03, -5.70).getPoint(alliance);
			Pose2d end = new MirroredPose(PlacementStates.getSwervePlacementTranslationAuto(Node.NODE8), new MirroredRotation(-90)).get(alliance);

			List<Translation2d> waypoints = Arrays.asList(
				// point1,//TODO uncomment this as well.
				point2
			);
		   return TrajectoryGenerator.generateTrajectory(start, waypoints, end, createTrajectoryConfigWithCableBumpSlowdown(speedConfig));
		}
	};


	public static final VariableAllianceTrajectory gamePiece1ToShoot3_1 = new VariableAllianceTrajectory() {
		@Override
		public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
			Pose2d start = new MirroredPose(PlacementConstants.gamePiece1SwerveLocation, new MirroredRotation(-70)).get(alliance);
			Pose2d end = new MirroredPose(PlacementConstants.shootLocation3, new MirroredRotation(-70)).get(alliance);
			
			List<Pose2d> waypoints = Arrays.asList(
				start,
				end			
			);

			return TrajectoryGenerator.generateTrajectory(waypoints, speedConfig);
		}
	};


	public static final VariableAllianceTrajectory shoot3ToGamePiece2_1 = new VariableAllianceTrajectory() {
		@Override
		public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
			Pose2d start = new MirroredPose(PlacementConstants.shootLocation3, new MirroredRotation(60)).get(alliance);
			Pose2d end = new MirroredPose(PlacementConstants.gamePiece2ShootPathLocation1, new MirroredRotation(60)).get(alliance);
			
			List<Pose2d> waypoints = Arrays.asList(
				start,
				end			
			);

			return TrajectoryGenerator.generateTrajectory(waypoints, speedConfig);
		}
	};


	public static final VariableAllianceTrajectory gamePiece2ToShoot5_1 = new VariableAllianceTrajectory() {
		@Override
		public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
			Pose2d start = new MirroredPose(PlacementConstants.gamePiece2ShootPathLocation1, new MirroredRotation(-70)).get(alliance);
			Pose2d end = new MirroredPose(PlacementConstants.shootLocation5, new MirroredRotation(-70)).get(alliance);
			
			List<Pose2d> waypoints = Arrays.asList(
				start,
				end			
			);

			return TrajectoryGenerator.generateTrajectory(waypoints, speedConfig);
		}
	};


	public static final VariableAllianceTrajectory shoot5ToGamePiece3_1 = new VariableAllianceTrajectory() {
		@Override
		public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
			Pose2d start = new MirroredPose(PlacementConstants.shootLocation5, new MirroredRotation(70)).get(alliance);
			Pose2d end = new MirroredPose(PlacementConstants.gamePiece3ShootPathLocation1, new MirroredRotation(70)).get(alliance);
			
			List<Pose2d> waypoints = Arrays.asList(
				start,
				end			
			);

		return TrajectoryGenerator.generateTrajectory(waypoints, speedConfig);
		}
	};


	public static final VariableAllianceTrajectory gamePiece3ToShoot7_1 = new VariableAllianceTrajectory() {
		@Override
		public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
			Pose2d start = new MirroredPose(PlacementConstants.gamePiece3ShootPathLocation1, new MirroredRotation(-70)).get(alliance);
		Pose2d end = new MirroredPose(PlacementConstants.shootLocation7, new MirroredRotation(-70)).get(alliance);
		
		List<Pose2d> waypoints = Arrays.asList(
			start,
			end			
		);

	   	return TrajectoryGenerator.generateTrajectory(waypoints, speedConfig);
		}
	};


	public static final VariableAllianceTrajectory shoot7ToGamePiece4_1 = new VariableAllianceTrajectory() {
		@Override
		public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
			Pose2d start = new MirroredPose(PlacementConstants.shootLocation7, new MirroredRotation(60)).get(alliance);
			Pose2d end = new MirroredPose(PlacementConstants.gamePiece4ShootPathLocation1, new MirroredRotation(60)).get(alliance);
			
			List<Pose2d> waypoints = Arrays.asList(
				start,
				end			
			);
	
		   return TrajectoryGenerator.generateTrajectory(waypoints, speedConfig);
		}
	};


	public static final VariableAllianceTrajectory gamePiece4ToShoot6_1 = new VariableAllianceTrajectory() {
		@Override
		public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
			Pose2d start = new MirroredPose(PlacementConstants.gamePiece4ShootPathLocation1, new MirroredRotation(-125)).get(alliance);
			Pose2d end = new MirroredPose(PlacementConstants.shootLocation6, new MirroredRotation(-125)).get(alliance);
			
			List<Pose2d> waypoints = Arrays.asList(
				start,
				end			
			);

		return TrajectoryGenerator.generateTrajectory(waypoints, speedConfig);
		}
	};


	public static final VariableAllianceTrajectory gamePiece4ToShoot8_9 = new VariableAllianceTrajectory() {
		@Override
		public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
			Pose2d start = new MirroredPose(PlacementConstants.gamePiece4ShootPathLocation9, new MirroredRotation(-130)).get(alliance);
			Pose2d end = new MirroredPose(PlacementConstants.shootLocation8, new MirroredRotation(-130)).get(alliance);
			
			List<Pose2d> waypoints = Arrays.asList(
				start,
				end
			);

		return TrajectoryGenerator.generateTrajectory(waypoints, speedConfig);
		}
	};


	public static final VariableAllianceTrajectory shoot8ToGamePiece3_9 = new VariableAllianceTrajectory() {
		@Override
		public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
			Pose2d start = new MirroredPose(PlacementConstants.shootLocation8, new MirroredRotation(120)).get(alliance);
			Pose2d end = new MirroredPose(PlacementConstants.gamePiece3ShootPathLocation9, new MirroredRotation(120)).get(alliance);
			
			List<Pose2d> waypoints = Arrays.asList(
				start,
				end			
			);
	
		   return TrajectoryGenerator.generateTrajectory(waypoints, speedConfig);
		}
	};


	public static final VariableAllianceTrajectory gamePiece3ToShoot5_9 = new VariableAllianceTrajectory() {
		@Override
		public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
			Pose2d start = new MirroredPose(PlacementConstants.gamePiece3ShootPathLocation9, new MirroredRotation(-115)).get(alliance);
			Pose2d end = new MirroredPose(PlacementConstants.shootLocation5, new MirroredRotation(-115)).get(alliance);
			
			List<Pose2d> waypoints = Arrays.asList(
				start,
				end			
			);

		return TrajectoryGenerator.generateTrajectory(waypoints, speedConfig);
		}
	};

	
	public static final VariableAllianceTrajectory gamePiece3ToShoot8_9 = new VariableAllianceTrajectory() {
		@Override
		public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
			Pose2d start = new MirroredPose(PlacementConstants.gamePiece3ShootPathLocation9, new MirroredRotation(-65)).get(alliance);
			Pose2d end = new MirroredPose(PlacementConstants.shootLocation8, new MirroredRotation(-65)).get(alliance);
			
			List<Pose2d> waypoints = Arrays.asList(
				start,
				end			
			);

		return TrajectoryGenerator.generateTrajectory(waypoints, speedConfig);
		}
	};


	public static final VariableAllianceTrajectory shoot5ToGamePiece2_9 = new VariableAllianceTrajectory() {
		@Override
		public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
			Pose2d start = new MirroredPose(PlacementConstants.shootLocation5, new MirroredRotation(115)).get(alliance);
			Pose2d end = new MirroredPose(PlacementConstants.gamePiece2ShootPathLocation9, new MirroredRotation(115)).get(alliance);
			
			List<Pose2d> waypoints = Arrays.asList(
				start,
				end
			);
	
		   return TrajectoryGenerator.generateTrajectory(waypoints, speedConfig);
		}
	};


	public static final VariableAllianceTrajectory gamePiece2ToShoot4_9 = new VariableAllianceTrajectory() {
		@Override
		public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
			Pose2d start = new MirroredPose(PlacementConstants.gamePiece2ShootPathLocation9, new MirroredRotation(-75)).get(alliance);
			Pose2d end = new MirroredPose(PlacementConstants.shootLocation4, new MirroredRotation(-90)).get(alliance);
			
			List<Pose2d> waypoints = Arrays.asList(
				start,
				end			
			);

		return TrajectoryGenerator.generateTrajectory(waypoints, speedConfig);
		}
	};


	public static final VariableAllianceTrajectory shoot9ToGamePiece2_9 = new VariableAllianceTrajectory() {
		@Override
		public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
			Pose2d start = new MirroredPose(PlacementConstants.shootLocation9, new MirroredRotation(135)).get(alliance);
			Pose2d end = new MirroredPose(PlacementConstants.gamePiece2ShootPathLocation9, new MirroredRotation(135)).get(alliance);
			
			List<Pose2d> waypoints = Arrays.asList(
				start,
				end			
			);
	
		   return TrajectoryGenerator.generateTrajectory(waypoints, speedConfig);
		}
	};


    public static final VariableAllianceTrajectory shoot8ToLeaveCommunity = new VariableAllianceTrajectory() {
		@Override
		public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
			Pose2d start = new MirroredPose(PlacementConstants.shootLocation8, new MirroredRotation(135)).get(alliance);
			Pose2d end = new MirroredPose(PlacementConstants.gamePiece3SwerveLocation, new MirroredRotation(135)).get(alliance);
			
			List<Pose2d> waypoints = Arrays.asList(
				start,
				end			
			);
	
		   return TrajectoryGenerator.generateTrajectory(waypoints, speedConfig);
		}
	};
}