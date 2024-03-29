package test;

import edu.wpi.first.math.geometry.Translation2d;
/**
    Takes points from pathplanner's coordinate system and converts it to our coordinate system. 
    Our system has 0,0 where the  white center line meets the wall on the feeder station side. 
    We have forwards(gyro angle) as 90 degrees on both sides, so on blue x is always positive, and on red, x is negative. negative y is on our side of the white line, 
    and positive y is on the opposing alliance's side of the whilte line.
 * 
 */
public class FieldPointCalculator {
    public static final Translation2d fieldSize = new Translation2d(8.0, 16.51);
    
    public static void main(String[] args) {
        // System.out.println(convertScreamCoordsToPathPlannerCoordsBlue(new Translation2d()));
        System.out.println(convertPathplannerCoordsToScreamCoords(new Translation2d(5.29, 0.97)));
    }


    public static Translation2d convertPathplannerCoordsToScreamCoords(Translation2d pathplannerCoords){
        return new Translation2d(fieldSize.getX()-pathplannerCoords.getY(), pathplannerCoords.getX()-(fieldSize.getY()/2.0));
    }

    public static Translation2d convertScreamCoordsToPathPlannerCoordsBlue(Translation2d screamCoords){//inverse function of pathplannerToScreamCoords
        return new Translation2d(screamCoords.getY()+(fieldSize.getY()/2), fieldSize.getX()-screamCoords.getX());
    }


}
