package frc2023.shuffleboard.tabs;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.team4522.lib.pid.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.GenericPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import frc2023.Constants;
import frc2023.Constants.SwerveModuleConstants;
import frc2023.shuffleboard.ShuffleboardTabBase;
import frc2023.subsystems.Swerve;
import frc2023.subsystems.Swerve.SwerveState;
import frc2023.swerve.SwerveRotationHelper;

public class SwerveTab extends ShuffleboardTabBase {

	private final Swerve mSwerve = Swerve.getInstance();

	private GenericPublisher mLastState;
	private GenericPublisher mDemandedState;
	private GenericPublisher mRotationHelperMode;

	private GenericEntry mCurrentDraw;
	private GenericEntry mVoltage;
	private GenericEntry mPowerConsumption;

	private GenericPublisher mSwerveX;
	private GenericPublisher mSwerveY;
	private GenericPublisher mSwerveTheta;

    private GenericEntry mSteerKP;
    private GenericEntry mSteerKD;
    private GenericEntry mSteerKF;
    private GenericEntry mSteerKS;

    private GenericEntry mDriveKP;
    private GenericEntry mDriveKD;
    private GenericEntry mDriveKF;
    private GenericEntry mDriveKS;

	private GenericEntry mFLEncoder;
	private GenericEntry mBLEncoder;
	private GenericEntry mBREncoder;
	private GenericEntry mFREncoder;
	
	private static SwerveTab mInstance = null;
	public static SwerveTab getInstance(){
		if(mInstance == null){
			mInstance = new SwerveTab();
		}
		return mInstance;
	}

	@Override
	public void createEntries() {
		mTab = Shuffleboard.getTab("Swerve");
		mTab.add("Set Brake Mode" , new InstantCommand(() -> mSwerve.setNeutralMode(NeutralMode.Brake, NeutralMode.Brake)));
		mTab.add("Set Coast Mode" , new InstantCommand(() -> mSwerve.setNeutralMode(NeutralMode.Coast, NeutralMode.Coast)));

		mLastState = createStringEntry("Last State", SwerveState.DISABLED.toString());
		mDemandedState = createStringEntry("Demanded State", SwerveState.DISABLED.toString());
		mRotationHelperMode = createStringEntry("Rotation Helper Mode", SwerveRotationHelper.RotationHelperMode.DISABLED.toString());
		
        mCurrentDraw = createNumberEntry("Current Draw (A)", 0.0);
        mVoltage = createNumberEntry("Voltage (V)", 0.0);
        mPowerConsumption = createNumberEntry("Power Consumption (W)", 0.0);

		mSwerveX = createNumberEntry("X", 0);
		mSwerveY = createNumberEntry("Y", 0);
		mSwerveTheta = createNumberEntry("Theta", 0);

		if(Constants.updatePIDsFromShuffleboard){
			mSteerKP = createNumberEntry("Steer kP", SwerveModuleConstants.DefaultConfig.steerConstants.kP());
			mSteerKD = createNumberEntry("Steer kD", SwerveModuleConstants.DefaultConfig.steerConstants.kD());
			mSteerKF = createNumberEntry("Steer kF", SwerveModuleConstants.DefaultConfig.steerConstants.kF());
			mSteerKS = createNumberEntry("Steer kS", SwerveModuleConstants.DefaultConfig.steerKS);
	
			mDriveKP = createNumberEntry("Drive kP", SwerveModuleConstants.DefaultConfig.steerConstants.kP());
			mDriveKD = createNumberEntry("Drive kD", SwerveModuleConstants.DefaultConfig.steerConstants.kD());
			mDriveKF = createNumberEntry("Drive kF", SwerveModuleConstants.DefaultConfig.steerConstants.kF());
			mDriveKS = createNumberEntry("Drive kS", SwerveModuleConstants.DefaultConfig.steerKS);
		}

		mFLEncoder = createNumberEntry(" FL Encoder Reading(degrees)", mSwerve.getModules()[0].getRawEncoder().getDegrees());
		mBLEncoder = createNumberEntry(" BL Encoder Reading(degrees)", mSwerve.getModules()[1].getRawEncoder().getDegrees());
		mBREncoder = createNumberEntry(" BR Encoder Reading(degrees)", mSwerve.getModules()[2].getRawEncoder().getDegrees());
		mFREncoder = createNumberEntry(" FR Encoder Reading(degrees)", mSwerve.getModules()[3].getRawEncoder().getDegrees());
	}

	@Override
	public void update() {
     	mLastState.setString(mSwerve.getLastState().toString());
		mDemandedState.setString(mSwerve.mPeriodicIO.state.toString());
		mRotationHelperMode.setString(mSwerve.getRotationHelperMode().toString());

		mCurrentDraw.setDouble(mSwerve.getCurrentDraw());
		mVoltage.setDouble(mSwerve.getVoltage());
		mPowerConsumption.setDouble(mSwerve.getPowerConsumption());

		Pose2d swervePose = mSwerve.getRobotPose();
		mSwerveX.setDouble(swervePose.getX());
		mSwerveY.setDouble(swervePose.getY());
		mSwerveTheta.setDouble(swervePose.getRotation().getDegrees());

		mFLEncoder.setDouble(mSwerve.getModules()[0].getRawEncoder().getDegrees());
		mBLEncoder.setDouble(mSwerve.getModules()[1].getRawEncoder().getDegrees());
		mBREncoder.setDouble(mSwerve.getModules()[2].getRawEncoder().getDegrees());
		mFREncoder.setDouble(mSwerve.getModules()[3].getRawEncoder().getDegrees());
	}
			
    public void updateDrivePIDConstants(){
        PIDConstants constants = new PIDConstants();
        constants.setkP(mDriveKP.getDouble(SwerveModuleConstants.DefaultConfig.driveConstants.kP()));
        constants.setkD(mDriveKD.getDouble(SwerveModuleConstants.DefaultConfig.driveConstants.kD()));
        constants.setkF(mDriveKF.getDouble(SwerveModuleConstants.DefaultConfig.driveConstants.kF()));
		mSwerve.updateDrivePIDConstants(constants);
		mSwerve.updateDriveKS(mDriveKS.getDouble(SwerveModuleConstants.DefaultConfig.driveKS));
    }

    public void updateSteerPIDConstants(){
        PIDConstants constants = new PIDConstants();
        constants.setkP(mSteerKP.getDouble(SwerveModuleConstants.DefaultConfig.steerConstants.kP()));
        constants.setkD(mSteerKD.getDouble(SwerveModuleConstants.DefaultConfig.steerConstants.kD()));
        constants.setkF(mSteerKF.getDouble(SwerveModuleConstants.DefaultConfig.steerConstants.kF()));
		mSwerve.updateSteerPIDConstants(constants);
		mSwerve.updateSteerKS(mSteerKS.getDouble(SwerveModuleConstants.DefaultConfig.steerKS));
    }
}