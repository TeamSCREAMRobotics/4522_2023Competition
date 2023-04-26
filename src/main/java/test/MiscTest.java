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
import frc2023.Constants.PlacementConstants;
import frc2023.Constants.SwerveConstants;
import frc2023.auto.Trajectories;
import frc2023.auto.modes.VariableSpeedTrajectory.TrajectorySpeed;
import frc2023.field.MirroredPose;
import frc2023.field.MirroredTranslation;
import frc2023.swerve.SwerveModule;

public class MiscTest {

 
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
