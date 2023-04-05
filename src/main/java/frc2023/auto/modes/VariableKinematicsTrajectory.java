package frc2023.auto.modes;

import java.util.HashMap;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc2023.Constants.SwerveConstants;

public abstract class VariableKinematicsTrajectory {
    
    private final HashMap<TrajectorySpeed, Trajectory> map = new HashMap<TrajectorySpeed, Trajectory>();

    public static enum TrajectorySpeed{
        SLOW, MEDIUM, FAST
    }
    
    public VariableKinematicsTrajectory (){
        map.put(TrajectorySpeed.SLOW, createTrajectories(SwerveConstants.defaultSlowSpeedConfig));
        map.put(TrajectorySpeed.MEDIUM, createTrajectories(SwerveConstants.defaultMediumSpeedConfig));
        map.put(TrajectorySpeed.FAST, createTrajectories(SwerveConstants.defaultFastSpeedConfig));
    }

    /**
     * DO NOT CALL THIS OUTSIDE OF TRAJECTORIES
     */
    protected abstract Trajectory createTrajectories(TrajectoryConfig kinematicsLimit);

    public Trajectory getTrajectory(TrajectorySpeed speed) {
        return map.get(speed);
    }
}