package frc2023.auto.modes;


import java.util.HashMap;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc2023.PlacementStates.Level;
import frc2023.auto.modes.VariableKinematicsTrajectory.TrajectorySpeed;

public abstract class VariableLevelTrajectory {

    private final HashMap<Level, VariableAllianceTrajectory> map = new HashMap<Level, VariableAllianceTrajectory>();

    public VariableLevelTrajectory(){

        map.put(Level.HYBRID, new VariableAllianceTrajectory(){
            @Override
            public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig kinematicsLimit) {
                return generateTrajectory(alliance, Level.HYBRID, kinematicsLimit);
            }
        });

        map.put(Level.MIDDLE, new VariableAllianceTrajectory(){
            @Override
            public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig kinematicsLimit) {
                return generateTrajectory(alliance, Level.MIDDLE, kinematicsLimit);
            }
        });

        map.put(Level.TOP, new VariableAllianceTrajectory(){
            @Override
            public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig kinematicsLimit) {
                return generateTrajectory(alliance, Level.TOP, kinematicsLimit);
            }
        });
    }

    /**
     * DO NOT CALL THIS OUTSIDE OF TRAJECTORIES
     */
    protected abstract Trajectory generateTrajectory(Alliance alliance, Level level, TrajectoryConfig kinematicsLimit);

    public Trajectory getTrajectory(Alliance alliance, Level level, TrajectorySpeed speed){
        return map.get(level).getTrajectory(alliance, speed);
    }
}
