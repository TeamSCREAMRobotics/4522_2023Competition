package frc2023.field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class MirroredPose {
    
    private MirroredTranslation mMirroredTranslation;
    private MirroredRotation mMirroredRotation;

    public MirroredPose(MirroredTranslation mirroredTranslation, MirroredRotation mirroredRotation){
        mMirroredTranslation = mirroredTranslation;
        mMirroredRotation = mirroredRotation;
    }

    public MirroredPose(double x, double y, double rotationDegrees){
        this(new MirroredTranslation(new Translation2d(x, y)), new MirroredRotation(Rotation2d.fromDegrees(rotationDegrees)));
    }

    public MirroredPose(double x, double y, Rotation2d rotation){
        this(new MirroredTranslation(x, y), new MirroredRotation(rotation));
    }
 
    public MirroredPose(double x, double y, MirroredRotation rotation){
        this(new MirroredTranslation(x, y), rotation);
    }

    public MirroredPose(Translation2d translation, MirroredRotation mirroredRotation) {
        this(new MirroredTranslation(translation), mirroredRotation);
    }

    public MirroredPose(MirroredTranslation mirroredTranslation, Rotation2d blueRotation) {
        this(mirroredTranslation, new MirroredRotation(blueRotation));
    }

    public MirroredPose(Translation2d translation, Rotation2d rotation) {
        this(new MirroredTranslation(translation), new MirroredRotation(rotation));
    }

    public Pose2d getPose(Alliance currentAlliance, Alliance pointAlliance){
            return new Pose2d(mMirroredTranslation.getPoint(currentAlliance, pointAlliance), mMirroredRotation.get(currentAlliance));
    }

    public MirroredTranslation getMirroredTranslation(){
        return mMirroredTranslation;
    }

    public MirroredRotation getMirroredRotation(){
        return mMirroredRotation;
    }

    public Pose2d get(Alliance alliance){
        return getPose(alliance, alliance);
    }

    public MirroredPose withRotation(MirroredRotation newRotation){
        return new MirroredPose(mMirroredTranslation, newRotation);
    }
}
