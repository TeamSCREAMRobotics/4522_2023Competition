package frc2023.auto.modes;

import java.util.HashMap;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc2023.auto.modes.VariableSpeedTrajectory.TrajectorySpeed;

public abstract class VariableAllianceTrajectory {
    
    private final HashMap<Alliance, VariableSpeedTrajectory> map = new HashMap<Alliance, VariableSpeedTrajectory>();

    public VariableAllianceTrajectory (){
      
        map.put(Alliance.Blue, new VariableSpeedTrajectory(){
            @Override
            protected Trajectory createTrajectories(TrajectoryConfig speedConfig) {
                return generateTrajectories(Alliance.Blue, speedConfig);
            }

        });

        map.put(Alliance.Red,  new VariableSpeedTrajectory(){
            @Override
            protected Trajectory createTrajectories(TrajectoryConfig speedConfig) {
                return generateTrajectories(Alliance.Red, speedConfig);
            }

        });
    }

    /**
     * DO NOT CALL THIS OUTSIDE OF TRAJECTORIES
     */
    protected abstract Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig speedConfig);

    public Trajectory getTrajectory(Alliance alliance, TrajectorySpeed trajectorySpeed) {
        if(alliance == Alliance.Invalid){
            DriverStation.reportWarning("invalid alliance in variableAllianceTrajectory", false);
        }
        return map.get(alliance).getTrajectory(trajectorySpeed);
    }
}
