package test;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc2023.swerve.SwerveModule;

public class MiscTest {
    public static void main(String[] args) {
      System.out.println(desiredStateToNativeState(new SwerveModuleState(1, Rotation2d.fromDegrees(45)), Rotation2d.fromDegrees(720)));
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
