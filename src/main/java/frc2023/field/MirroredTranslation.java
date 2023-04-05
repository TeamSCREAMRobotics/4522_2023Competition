package frc2023.field;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;


public class MirroredTranslation {
    
    private Translation2d mBluePoint, mRedPoint;
    private Translation2d mBlueOriginalPosition, mBlueOffset, mRedOffset;
    
    /**
     * @param bluePosition The position of the blue alliance coordinate
     * @param blueOffset The offset to the point on the blue side. The offset only applies to the coordinate on the blue side
     * @param redOffset The offset to the point on the red side. The offset only applies to the coordinate on the red side
     */
    public MirroredTranslation(Translation2d blueOriginalPosition, Translation2d blueOffset, Translation2d redOffset){
        mBlueOriginalPosition = blueOriginalPosition;
        configOffsets(blueOffset, redOffset);
    }

    public MirroredTranslation(Translation2d blueOriginalPosition){
        this(blueOriginalPosition, new Translation2d(), new Translation2d());
    }

    private void updateOffsets(){
        mBluePoint = mBlueOriginalPosition.plus(mBlueOffset);
        mRedPoint = new Translation2d(-mBlueOriginalPosition.getX(), mBlueOriginalPosition.getY()).plus(mRedOffset);
    }

    public MirroredTranslation(double x, double y){
        this(new Translation2d(x, y));
    }

    public Translation2d getPoint(Alliance currentAlliance, Alliance pointAlliance){
        if(pointAlliance == Alliance.Blue && currentAlliance == Alliance.Blue){
            return mBluePoint;
        } else if(pointAlliance == Alliance.Red && currentAlliance == Alliance.Red){
            return mRedPoint;
        } else if(pointAlliance == Alliance.Blue && currentAlliance == Alliance.Red){
            return new Translation2d(-mBluePoint.getX(), -mBluePoint.getY());
        } else{//pointAlliance == Alliance.RED && currentAlliance == Alliance.Blue
            return new Translation2d(-mRedPoint.getX(), -mRedPoint.getY());
        }
    }

    public Translation2d getPoint(Alliance alliance){
        return getPoint(alliance, alliance);
    }

    public MirroredTranslation plus(Translation2d additionToBlue){
        return new MirroredTranslation(mBluePoint.plus(additionToBlue), mBlueOffset, mRedOffset);//there is a better way to do this that doesn't store all of the offsets for every point.
    }

    public MirroredTranslation minus(Translation2d subtractionFromBlue){
        return new MirroredTranslation(mBluePoint.minus(subtractionFromBlue), mBlueOffset, mRedOffset);//there is a better way to do this that doesn't store all of the offsets for every point.
    }

    public void offset(Translation2d offset){
        mBlueOriginalPosition = mBlueOriginalPosition.plus(offset);
        updateOffsets();
    }

    public void configOffsets(Translation2d blueOffset, Translation2d redOffset){
        mBlueOffset = blueOffset;
        mRedOffset = redOffset;
        updateOffsets();
    }

    @Override
    public String toString() {
        return "MirroredTranslation{" + mBluePoint + "   red: " + mRedPoint + " }";
    }
}