# 4522CompetitionCode2023
 

## Code Features
* Device checking system that ensures devices are configured properly. Prints errors to the driverstation if a device cannot be configured

* Limelight-based vision updates for pose estimation

     Front and back limelights, with apriltag and retroreflective vision tape based pose estimation. Filtering based on robot speeds to discard measurements if there is motion blur.
     
* Automatic placement based on robot's pose estimate

* Trajectory generation with WPILib trajectory generator

     For each path segment, we automatically generate slow, medium, and fast speed trajectories in our VariableSpeedTrajectory class. While this means we generate significantly more trajectories than we actually use, we can very easily change our trajectories. If, for example, a trajectory is moving too slowly and we want to change it easily during compeition, all we have to do is change the speed variable where we select the trajectory.
     
     We also have a constraint to limit the speed of the robot while going over the cable bump during trajectory.
     
     //TODO
     
 *Hotswap Modules
 * 3 piece autobalance on both sides
 * Special coast auto
 *Mirrored coordinate system, but able to hold offsets for both alliances
 * Swerve module && arm controlled with onboard motionmagic/PID
 * Config class for every device to simplify setting status frames(part of device checker
 * Custom action(commands) setup, with superstructure setup to run autos in teleop
 
 
 ## Robot Mechanical Overview
 
 //TODO
 
 ## Robot Code Structure
 
 //TODO
 
## Notable Files
- [`./Robot.java`]()
- [`./subsystems/`](src/main/java/frc2023/subsystems)
	- [`Superstructure.java`](src/main/java/frc2023/subsystems/Superstructure.java) - All Subsystem logic, auto routine execution during teleop
	- [`Swerve.java`](src/main/java/frc2023/subsystems/Swerve.java) - Swerve drive logic and state control
  -[`SwerveDriveHelper.java`](src/main/java/frc2023/swerve/SwerveDriveHelper.java) - Swerve drive math helper
	- [`Limelight.java`](src/main/java/frc2023/subsystems/Limelight.java) - Front and back limelights, vision updates from apriltags and vision tape
	- [`Intake.java`](src/main/java/frc2023/subsystems/Intake.java) - Subsytem that intakes cubes, and can shoot them out of the front or back of the robot
- [`./auto/`](src/main/java/frc2023/auto)
	- [`Trajectories.java`](src/main/java/frc2023/auto/Trajectories.java) - Contains all auto trajectories
	- [`AutoSegments.java`](src/main/java/frc2023/auto/AutoSegments.java) - Each segment that out autos use as building blocks
	- [`AutoRoutines.java`](src/main/java/frc2023/auto/AutoRoutines.java) - All of our autos
	- [`AutoModeExecutor.java`](src/main/java/frc2023/auto/modes/AutoRoutineExecutor.java) -  Custom executor for Actions.
 
//TODO link all of the device checker classes
//TODO link the variable trajectories, swervemodule, arm, controlboard, maybe the rotation helper


 
 
 ## Credits & References Used
- FRC Team 254's SwerveSetpointGenerator
- FRC Team 6328's PoseEstimator
- FRC Team 1678's Superstructure, ControlBoard setup, && README
     
     //TODO finish README
