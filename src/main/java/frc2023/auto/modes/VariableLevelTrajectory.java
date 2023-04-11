package frc2023.auto.modes;


import java.util.HashMap;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc2023.PlacementStates.Level;
import frc2023.auto.modes.VariableSpeedTrajectory.TrajectorySpeed;

public abstract class VariableLevelTrajectory {

    private final HashMap<Level, VariableAllianceTrajectory> map = new HashMap<Level, VariableAllianceTrajectory>();

    public VariableLevelTrajectory(){

        map.put(Level.HYBRID, new VariableAllianceTrajectory(){
            @Override
            public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
                return generateTrajectory(alliance, Level.HYBRID, speedConfig);
            }
        });

        map.put(Level.MIDDLE, new VariableAllianceTrajectory(){
            @Override
            public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
                return generateTrajectory(alliance, Level.MIDDLE, speedConfig);
            }
        });

        map.put(Level.TOP, new VariableAllianceTrajectory(){
            @Override
            public Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig) {
                return generateTrajectory(alliance, Level.TOP, speedConfig);
            }
        });
    }

    /**
     * DO NOT CALL THIS OUTSIDE OF TRAJECTORIES
     */
    protected abstract Trajectory generateTrajectory(Alliance alliance, Level level, TrajectoryConfig speedConfig);

    public Trajectory getTrajectory(Alliance alliance, Level level, TrajectorySpeed speed){
        return map.get(level).getTrajectory(alliance, speed);
    }
}
