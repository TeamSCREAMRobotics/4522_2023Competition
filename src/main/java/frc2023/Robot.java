package frc2023;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team4522.lib.PeriodicLoop;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import frc2023.Constants.ArmConstants;
import frc2023.Constants.VisionConstants;
import frc2023.auto.Trajectories;
import frc2023.auto.modes.AutoRoutineExecutor;
import frc2023.controlboard.ControlBoard;
import frc2023.shuffleboard.ShuffleboardTabManager;
import frc2023.shuffleboard.tabs.MatchTab;
import frc2023.subsystems.Arm;
import frc2023.subsystems.Devices;
import frc2023.subsystems.Gripper;
import frc2023.subsystems.Intake;
import frc2023.subsystems.Limelight;
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
  private final Limelight mFrontLimelight;
  private final Limelight mBackLimelight;
  private final PowerDistribution mPDH = new PowerDistribution(Ports.pdhID, ModuleType.kRev);
  private final ShuffleboardTabManager mShuffleboardTabManager;
  private final MatchTab mMatchTab;

  private final PeriodicLoop mOdometryLoop;
  private final PeriodicLoop mTelemetryLoop;
  private final PeriodicLoop mUpdatePIDsFromShuffleboard;

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
    mMatchTab = mShuffleboardTabManager.matchTab;
    mFrontLimelight = Limelight.getFrontInstance();
    mBackLimelight = Limelight.getBackInstance();

    mOdometryLoop = new PeriodicLoop(() -> mSwerve.updateOdometry(), Constants.kOdometryPeriodSeconds);
    mTelemetryLoop = new PeriodicLoop(() -> mSubsystemManager.outputTelemetry(), Constants.kTelemetryPeriodSeconds);

    mUpdatePIDsFromShuffleboard = new PeriodicLoop(() -> {
      // mShuffleboardTabManager.mSwerveTab.updateDrivePIDConstants();//TODO last time this was used, something weird happened
      // mShuffleboardTabManager.mSwerveTab.updateSteerPIDConstants();
      // mShuffleboardTabManager.mArmTab.updatePivotPIDConstants();
      // mShuffleboardTabManager.mArmTab.updateTelescopePIDConstants();
      mShuffleboardTabManager.testTab.updateAutoBalancePIDConstants();

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
    mBackLimelight.setPipeline(VisionConstants.kRobotBootedUpPipeline);
    mBackLimelight.writeOutputs();

    DriverStation.reportWarning("Robot Initialized", false);
  }


  @Override
  public void robotPeriodic() {
    mShuffleboardTabManager.update();
  }


  @Override
  public void autonomousInit() {
    mAutoRoutineExecutor.selectRoutine(mMatchTab.getSelectedAutoRoutine());
    mAutoRoutineExecutor.start();
  }


  @Override
  public void autonomousPeriodic() {
    mAutoRoutineExecutor.execute();

    if(mControlBoard.getArmManualOverride()) mArm.disable();

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
    mSubsystemManager.stop();
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
    mBackLimelight.setPipeline(0);
  }
  

  @Override
  public void testInit() {
    mArm.configTelescopeSoftLimitsEnabled(false);//We disable the soft limits in test mode to test the arm hard stops, and we reenable soft limits in enabledInit.
    mArm.configPivotSoftLimitsEnabled(false);
  }


  @Override
  public void testPeriodic() {
    if(mControlBoard.getArmManualOverride()){      
				double pivotPO = mControlBoard.getPivotPO();
				double telescopePO = mControlBoard.getTelescopePO();
				double multiplier = (mControlBoard.getArmManualSlowMode()? 0.5 : 1);
				mArm.setPercentOutput(pivotPO*multiplier, telescopePO*multiplier);
    } else mArm.setPosition(new Translation2d(ArmConstants.kMinTelescopeLength+0.03, Rotation2d.fromDegrees(-90)), true);

    if(mControlBoard.getZeroTelescope()){
      mArm.zeroTelescope();
    }
  
    mSubsystemManager.writeOutputs();
  }
}