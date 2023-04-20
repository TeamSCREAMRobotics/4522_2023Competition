package test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.EllipticalRegionConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.math.trajectory.constraint.TrajectoryConstraint.MinMax;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc2023.Constants.FieldConstants;
import frc2023.Constants.SwerveConstants;
import frc2023.auto.Trajectories;
import frc2023.auto.modes.VariableSpeedTrajectory.TrajectorySpeed;
import frc2023.field.MirroredTranslation;
import frc2023.swerve.SwerveModule;

public class MiscTest {

  
  public static final MirroredTranslation cableBump = new MirroredTranslation(7.27, -4.38);
  public static final double kAutoSlowdownForCableBumpRadius = 1.35;// meters

	
	public static TrajectoryConfig createTrajectoryConfigWithCableBumpSlowdown(TrajectoryConfig config){
		return createTrajectoryConfigWithSlowdownArea(config, SwerveConstants.nearCableBumpConstraint, FieldConstants.cableBump.getPoint(DriverStation.getAlliance()), SwerveConstants.kAutoSlowdownForCableBumpRadius);
	}

	
	private static TrajectoryConfig createTrajectoryConfigWithSlowdownArea(TrajectoryConfig config, TrajectoryConstraint constraint, Translation2d point, double radius){
		return config.addConstraint(new EllipticalRegionConstraint(point, radius*2, radius*2, Rotation2d.fromDegrees(0), constraint));
	}
		


  // public static final TrajectoryConstraint nearCableBumpConstraint = new TrajectoryConstraint() {

  //   @Override
  //   public double getMaxVelocityMetersPerSecond(Pose2d poseMeters, double curvatureRadPerMeter,
  //       double velocityMetersPerSecond) {
  //     return 0.75;
  //   }//2.6531339212914573

  //   @Override
  //   public MinMax getMinMaxAccelerationMetersPerSecondSq(Pose2d poseMeters, double curvatureRadPerMeter,
  //       double velocityMetersPerSecond) {
  //     return new MinMax(-3.0, 3.0);
  //   }
  // };
    public static void main(String[] args) {

      System.out.println(createTrajectoryConfigWithCableBumpSlowdown(SwerveConstants.defaultFastSpeedConfig).getMaxVelocity());

      // System.out.println(desiredStateToNativeState(new SwerveModuleState(1, Rotation2d.fromDegrees(45)), Rotation2d.fromDegrees(720)));
    }



    private static SwerveModuleState desiredStateToNativeState(SwerveModuleState desiredState, Rotation2d currentAngle){
      double deltaDegrees = desiredState.angle.minus(currentAngle).getDegrees();
      boolean invert = Math.abs(deltaDegrees) > 90;
      if(invert) deltaDegrees -= Math.signum(deltaDegrees)*180;
  
      return new SwerveModuleState(
        (invert ? -1 : 1) * desiredState.speedMetersPerSecond,
        Rotation2d.fromDegrees(currentAngle.getDegrees() + deltaDegrees)
      );
    }
  
}
