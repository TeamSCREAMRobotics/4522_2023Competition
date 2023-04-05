package frc2023.field;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class MirroredRotation {
    private Rotation2d mBlueRotation, mRedRotation;

    public MirroredRotation(Rotation2d blueRotation){
        mBlueRotation = blueRotation;
        mRedRotation = new Rotation2d(-mBlueRotation.getCos(), mBlueRotation.getSin());
    }

    public MirroredRotation(double degrees){
        this(Rotation2d.fromDegrees(degrees));
    }

    public static Rotation2d get(double degrees, Alliance alliance){
        return get(Rotation2d.fromDegrees(degrees), alliance);
    }

    public static Rotation2d get(Rotation2d rotation, Alliance alliance){
        Rotation2d blueRotation = rotation;
        if(alliance == Alliance.Blue) return blueRotation;

        if(alliance == Alliance.Red) return new Rotation2d(-blueRotation.getCos(), blueRotation.getSin());
        DriverStation.reportError("error in MirroredRotation.get", false);
        return blueRotation;
    }

    public Rotation2d get(Alliance currentAlliance){
        if(currentAlliance == Alliance.Blue){
            return mBlueRotation;
        } else if(currentAlliance == Alliance.Red){
            return mRedRotation;
        } else{
            DriverStation.reportError("Issue with field rotation", false);
            return mBlueRotation;
        }
    }
}
