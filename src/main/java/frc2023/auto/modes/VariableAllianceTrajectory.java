package frc2023.auto.modes;

import java.util.HashMap;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc2023.auto.modes.VariableKinematicsTrajectory.TrajectorySpeed;

public abstract class VariableAllianceTrajectory {
    
    private final HashMap<Alliance, VariableKinematicsTrajectory> map = new HashMap<Alliance, VariableKinematicsTrajectory>();

    public VariableAllianceTrajectory (){
      
        map.put(Alliance.Blue, new VariableKinematicsTrajectory(){
            @Override
            protected Trajectory createTrajectories(TrajectoryConfig kinematicsLimit) {
                return generateTrajectories(Alliance.Blue, kinematicsLimit);
            }

        });

        map.put(Alliance.Red,  new VariableKinematicsTrajectory(){
            @Override
            protected Trajectory createTrajectories(TrajectoryConfig kinematicsLimit) {
                return generateTrajectories(Alliance.Red, kinematicsLimit);
            }

        });
    }

    /**
     * DO NOT CALL THIS OUTSIDE OF TRAJECTORIES
     */
    protected abstract Trajectory generateTrajectories(Alliance alliance, TrajectoryConfig kinematicsLimit);

    public Trajectory getTrajectory(Alliance alliance, TrajectorySpeed trajectorySpeed) {
        if(alliance == Alliance.Invalid){
            DriverStation.reportWarning("invalid alliance in variableAllianceTrajectory", false);
        }
        return map.get(alliance).getTrajectory(trajectorySpeed);
    }
}
