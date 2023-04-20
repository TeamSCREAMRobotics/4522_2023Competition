package frc2023;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team4522.lib.PeriodicLoop;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import frc2023.Constants.ArmConstants;
import frc2023.Constants.VisionConstants;
import frc2023.Constants.VisionConstants.BackLimelightConstants;
import frc2023.auto.AutoRoutines;
import frc2023.auto.Trajectories;
import frc2023.auto.actions.lib.ActionBase;
import frc2023.auto.actions.lib.EmptyAction;
import frc2023.auto.modes.AutoRoutineExecutor;
import frc2023.auto.modes.VariableSpeedTrajectory.TrajectorySpeed;
import frc2023.controlboard.ControlBoard;
import frc2023.shuffleboard.ShuffleboardTabManager;
import frc2023.subsystems.Arm;
import frc2023.subsystems.BackLimelight;
import frc2023.subsystems.Devices;
import frc2023.subsystems.Gripper;
import frc2023.subsystems.Intake;
import frc2023.subsystems.FrontLimelight;
import frc2023.subsystems.SubsystemManager;
import frc2023.subsystems.Superstructure;
import frc2023.subsystems.Swerve;

public class Robot extends TimedRobot {

  private final SubsystemManager mSubsystemManager;
  private final Superstructure mSuperstructure;
  private final AutoRoutineExecutor mAutoRoutineExecutor;
  private final Swerve mSwerve;
  private final ControlBoard mControlBoard;
  private final Arm mArm;
  private final Intake mIntake;
  private final Gripper mGripper;
  private final FrontLimelight mFrontLimelight;
  private final BackLimelight mBackLimelight;
  private final PowerDistribution mPDH = new PowerDistribution(Ports.pdhID, ModuleType.kRev);
  private final ShuffleboardTabManager mShuffleboardTabManager;

  private final PeriodicLoop mOdometryLoop;
  private final PeriodicLoop mTelemetryLoop;
  private final PeriodicLoop mUpdatePIDsFromShuffleboard;


  /**
   * We load all of our classes in a specific order on startup because our devices must be created before we use them. At the beginning of the season, we had an
   * issue with our swerve modules because we would sometimes zero the angle motor before the CANCoder could read properly. This DI setup ensures that this never happens.
   */
  public Robot(){
    super();
    LiveWindow.disableAllTelemetry();
    Devices.getInstance();
    Trajectories.getInstance();//this is called so that the trajectories generate on startup instead of when the auto is called
    mSwerve = Swerve.getInstance();
    mArm = Arm.getInstance();
    mIntake = Intake.getInstance();
    mGripper = Gripper.getInstance();
    mControlBoard = ControlBoard.getInstance();
    mSuperstructure = Superstructure.getInstance();
    mSubsystemManager = SubsystemManager.getInstance();
    mAutoRoutineExecutor = AutoRoutineExecutor.getInstance();
    mShuffleboardTabManager = ShuffleboardTabManager.getInstance();
    mFrontLimelight = FrontLimelight.getInstance();
    mBackLimelight = BackLimelight.getInstance();

    mOdometryLoop = new PeriodicLoop(() -> mSwerve.updateOdometry(), Constants.kOdometryPeriodSeconds);
    mTelemetryLoop = new PeriodicLoop(() -> mSubsystemManager.outputTelemetry(), Constants.kTelemetryPeriodSeconds);

    mUpdatePIDsFromShuffleboard = new PeriodicLoop(() -> {
      // mShuffleboardTabManager.mSwerveTab.updateDrivePIDConstants();//TODO last time this was used, something weird happened
      // mShuffleboardTabManager.mSwerveTab.updateSteerPIDConstants();
      // mShuffleboardTabManager.mArmTab.updatePivotPIDConstants();
      // mShuffleboardTabManager.mArmTab.updateTelescopePIDConstants();
      mShuffleboardTabManager.testTab.updateAutoBalancePIDConstants();
      mShuffleboardTabManager.testTab.updateRotationHelperPIDConstants();

    }, Constants.kUpdatePIDsFromShuffleboardPeriodSeconds);

    mSubsystemManager.setSubsystems(mSwerve, mIntake, mGripper, mArm, mFrontLimelight, mBackLimelight);

    mPDH.clearStickyFaults();
  }


  @Override
  public void robotInit() {
    mOdometryLoop.start();
    if(Constants.outputTelemetry) mTelemetryLoop.start();
    if(Constants.updatePIDsFromShuffleboard) mUpdatePIDsFromShuffleboard.start();

    //This is how we zero the arm for every match. The pivot and telescope zero on startup, and we turn on the back limelight LEDs as a status indicator to show when the bootup is complete
    mArm.zeroTelescope();
    mArm.resetPivotToAngle(Rotation2d.fromDegrees(-90));
    mIntake.zeroRodMotor();
    mBackLimelight.setPipeline(BackLimelightConstants.kRobotBootedUpPipeline);
    mBackLimelight.writeOutputs();

    DriverStation.reportWarning("Robot Initialized", false);
  }


  @Override
  public void robotPeriodic() {
    mShuffleboardTabManager.update();
  }


  @Override
  public void autonomousInit() {
    mAutoRoutineExecutor.selectRoutine(mShuffleboardTabManager.matchTab.getSelectedAutoRoutine());
    mAutoRoutineExecutor.start();
  }


  @Override
  public void autonomousPeriodic() {
    mAutoRoutineExecutor.execute();

    if(mControlBoard.getArmManualOverride()) mArm.disable();//TODO there may be a cleaner way of disabling the arm

    mSubsystemManager.writeOutputs();
  }


  @Override
  public void teleopInit() {
    
  }


  @Override
  public void teleopPeriodic() {
    mSuperstructure.updateTeleopCommands();
    mSubsystemManager.writeOutputs();
  }
  

  Timer mTimeSinceDisabled = new Timer();

  @Override
  public void disabledInit() {
    mAutoRoutineExecutor.stop();
    mSubsystemManager.stopAllSubsystems();
    mTimeSinceDisabled.reset();
    mTimeSinceDisabled.start();
  }


  @Override
  public void disabledPeriodic() {
    if(mTimeSinceDisabled.get() > 5.0 ){
      mArm.setNeutralMode(NeutralMode.Coast, NeutralMode.Coast);
      mSwerve.setNeutralMode(NeutralMode.Coast, NeutralMode.Coast);
    }

    mBackLimelight.writeOutputs();
  }


  @Override
  public void disabledExit() {
    System.out.println("ENABLED_INIT");
    mArm.configTelescopeSoftLimitsEnabled(true);
    mArm.configPivotSoftLimitsEnabled(true);
    mArm.setNeutralMode(NeutralMode.Brake, NeutralMode.Brake);
    mSwerve.setNeutralMode(NeutralMode.Brake, NeutralMode.Brake);
    mBackLimelight.setPipeline(BackLimelightConstants.kSubstationTagPipeline);
  }
  

  @Override
  public void testInit() {
    mArm.configTelescopeSoftLimitsEnabled(false);//We disable the soft limits in test mode to test the arm hard stops, and we reenable soft limits in enabledInit.
    mArm.configPivotSoftLimitsEnabled(false);
  }


  @Override
  public void testPeriodic() {
    if(mControlBoard.getArmManualOverride()){      
				// double pivotPO = mControlBoard.getPivotPO();
				// double telescopePO = mControlBoard.getTelescopePO();
				// double multiplier = (mControlBoard.getArmManualSlowMode()? 0.5 : 1);
				// mArm.setPercentOutput(pivotPO*multiplier, telescopePO*multiplier);
    } else mArm.setPosition(new Translation2d(ArmConstants.kMinTelescopeLength+0.03, Rotation2d.fromDegrees(-90)), true);

    if(mControlBoard.getZeroTelescope()){
      mArm.zeroTelescope();
    }
    // if(mControlBoard.getSlowMode()){
    //   mIntake.extendRod();
    // } else{
    //   mIntake.disable();//TODO disable shouldnt move rod
    // }
    // Devices.getInstance().dRodMotor.set(ControlMode.PercentOutput, mControlBoard.getPivotPO()/2.0);
    
    // System.out.println("  PO: " +mControlBoard.getPivotPO()/5.0 + "  pos: " + Devices.getInstance().dRodMotor.getSelectedSensorPosition());
    // System.out.print("    RODANGLE: " + mIntake.getRodAngle().getDegrees());
    mSubsystemManager.writeOutputs();
  }
}