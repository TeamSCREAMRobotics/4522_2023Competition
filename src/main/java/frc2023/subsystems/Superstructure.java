package frc2023.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import frc2023.PlacementStates;
import frc2023.Constants.ControlBoardConstants;
import frc2023.Constants.SwerveConstants;
import frc2023.PlacementStates.Level;
import frc2023.PlacementStates.Node;
import frc2023.auto.actions.autonomous.MoveToPoseAction;
import frc2023.auto.actions.autonomous.autoPlacement.LimelightAutoScore;
import frc2023.auto.actions.lib.ActionBase;
import frc2023.auto.actions.lib.SeriesAction;
import frc2023.auto.modes.AutoRoutineExecutor;
import frc2023.controlboard.ControlBoard;
import frc2023.subsystems.Swerve.DodgeDirection;
import frc2023.subsystems.Swerve.SwerveState;
import frc2023.swerve.SwerveRotationHelper.RotationHelperMode;

public class Superstructure {

	private final ControlBoard mControlBoard = ControlBoard.getInstance();
	private final Swerve mSwerve = Swerve.getInstance();
	private final AutoRoutineExecutor mAutoRoutineExecutor = AutoRoutineExecutor.getInstance();
	private final Intake mIntake = Intake.getInstance();
	private final Gripper mGripper = Gripper.getInstance();
	private final Arm mArm = Arm.getInstance();
	private final Limelight mFrontLimelight = Limelight.getFrontInstance();
	private final Limelight mBackLimelight = Limelight.getBackInstance();
	private final SubsystemManager mSubsystemManager = SubsystemManager.getInstance();


	private Superstructure(){
		mTimerSinceSwerveEnabled.reset();
		mTimerSinceSwerveEnabled.start();
	}


	private static Superstructure mInstance = null;
	public static Superstructure getInstance(){
		if(mInstance == null){
			mInstance = new Superstructure();
		}
		return mInstance;
	}


	private Optional<ActionBase> mSelectedRoutine = Optional.empty();
	private final Timer mTimerSinceSwerveEnabled = new Timer();

	public void updateTeleopCommands(){
		mSubsystemManager.disableAllSubsystems();

		mControlBoard.update();
		//ControlBoard values
		final Level selectedLevel = mControlBoard.getSelectedLevel();
		final Node selectedNode = mControlBoard.getSelectedNode();
		final Alliance alliance = DriverStation.getAlliance();

		mFrontLimelight.setVisionTargetNode(selectedNode);//tells the limelight which node we are targeting, because the retroreflective tapes all look the same and it is ambiguous which one the limelight sees

		//Automated Routine Logic(still in teleop, but automated)
		Optional<ActionBase> desiredAutoRoutine = Optional.empty();
	 	if(mControlBoard.getFullAutoPlace()){//our pose estimation via apriltags is not reliable enough for this to be better than the auto place without the position segment
			desiredAutoRoutine = Optional.of(new SeriesAction(
				new MoveToPoseAction(PlacementStates.getSwerveBackupBeforePlaceState(selectedNode, alliance), SwerveConstants.positionErrorForLimelightVisionTargetMode, SwerveConstants.angleErrorForLimelightVisionTargetMode),
				new LimelightAutoScore(selectedNode, selectedLevel, false)
			));
		} else if(mControlBoard.getAutoPlaceWithoutPosition()){
			desiredAutoRoutine = Optional.of(new LimelightAutoScore(selectedNode, selectedLevel, true));
		} 
		
		if(shouldSelectNewAutoRoutine(desiredAutoRoutine)) selectNewAutoRoutine(desiredAutoRoutine);

		
		final boolean autoControl = mSelectedRoutine.isPresent();

		if(autoControl){//if we are running an auto routine, execute the auto command
			mAutoRoutineExecutor.execute();
		} else {
			final boolean eject = mControlBoard.getEject();
			final boolean intake = mControlBoard.getIntake() && !eject;
			final boolean autoShoot = mControlBoard.getAutoShoot() && !intake;
			final boolean snapAndShoot = mControlBoard.getSnapAngleAndShoot() && !autoShoot;
			final boolean poopShootFromChargeLine = mControlBoard.getPoopShootFromChargeLine() && !snapAndShoot;
			final boolean prepareShooterForShot = mControlBoard.getPrepareIntakeForShot() && !poopShootFromChargeLine;
			final boolean shoot = !prepareShooterForShot && mControlBoard.getShootCube();
			final boolean sweep = !shoot && mControlBoard.getSweep();
			final boolean retract = !intake && !eject && !shoot && !sweep && !autoShoot && !snapAndShoot && !poopShootFromChargeLine && !prepareShooterForShot;
			final boolean backwardsEject = mControlBoard.getBackwardsEject() && retract;

			if(mControlBoard.armPlaceCone()){//moves the arm up to place at the selected position
				mArm.setPlacementPosition(selectedLevel);
			} else if(mControlBoard.getPreparePlacement()){//pulls the arm up to be closer to the placement state. We don't use this because the pivot is fast enough that it works fine from all the ways in the robot
				mArm.preparePlacement(selectedNode.isCone());
			} else if(intake || eject || shoot || sweep || snapAndShoot || autoShoot){//if we stick the intake out, we have to pull the arm up, because otherwise it sticks slightly out of the frame perimeter
				mArm.setPoopShootPosition();
			} else if(mControlBoard.getArmHoldSetpoint()){//sticks the arm to the side so that the cone can be indexed with the gripper properly
				mArm.setConeHold();
			} else{//if we don't tell the arm to do anything, we move it down to get ready to grab a cone, or if we are in cube mode we hold it down to lower our center of gravity.
				if(selectedNode.isCone()){
					mArm.retrieveCone();
				} 
				else if(selectedNode.isCube()){
					mArm.retrieveCube();
				}
			}


			//Gripper
			if(mControlBoard.getOpenGripper()) mGripper.open();
			else if(mControlBoard.getCloseGripper()) mGripper.close();

			//Eject
			if(eject){
				mIntake.eject();
			}

			//Shoot
			if(shoot){
				mIntake.shootCube(selectedLevel);
			}

			//Prepare shooter for shot
			if(prepareShooterForShot){
				mIntake.prepareForShot();
			}
			
			//Snap and shoot
			if(snapAndShoot){
				if(mSwerve.atAngleReference(SwerveConstants.robotForwardAngle, Rotation2d.fromDegrees(15))){
					mIntake.shootCube(selectedLevel);
				} else{
					mIntake.ejectOnlyLowerConveyor();
				}
			}

			//Sweep
			if(sweep){
				mIntake.sweep();
			}

			//Intake
			if(intake){
				mIntake.intake();
			}

			//Retract
			if(retract){
				mIntake.retract();
			}

			if(poopShootFromChargeLine){
				mIntake.poopShootFromChargeLine();
				mArm.setPoopShootPosition();
			}
			
			//Eject out the shooter side
			if(backwardsEject){
				mIntake.backwardsEject();
			}

			//Swerve
			if(mControlBoard.getZeroGyro()) mSwerve.resetRotation(SwerveConstants.robotForwardAngle);
			if(mControlBoard.getZeroPose()) mSwerve.resetPose(PlacementStates.getSwervePlacementPose(selectedNode, alliance));

			Optional<Rotation2d> targetAngle = mControlBoard.getSwerveTargetAngle();
			if(targetAngle.isPresent()){
				mSwerve.setSnapAngle(targetAngle.get());
			}

			Translation2d swerveTranslation = mControlBoard.getSwerveTranslation();
			Rotation2d swerveRotation = mControlBoard.getSwerveRotation();
			boolean robotCentric = mControlBoard.getRobotCentric();

			if(mSwerve.getLastState() != SwerveState.DISABLED && mSwerve.getLastState() != SwerveState.LOCK_WHEELS){
				mTimerSinceSwerveEnabled.reset();
				mTimerSinceSwerveEnabled.start();
			}

			if(mControlBoard.getSlowMode()) {
				swerveTranslation = swerveTranslation.times(mControlBoard.getSlowModeTranslationScalar());
				swerveRotation = swerveRotation.times(mControlBoard.getSlowModeRotationScalar());
			}

			final boolean normalDriveRequested = ((swerveTranslation.getNorm() > 0.01) || (Math.abs(swerveRotation.getRadians()) > 0.01)  || (mSwerve.mSwerveDriveHelper.getRotationMode() == RotationHelperMode.SNAP));
			
			if(poopShootFromChargeLine){
				mSwerve.driveAndFaceAngle(swerveTranslation, SwerveConstants.robotBackwardAngle, robotCentric);
			} else if(snapAndShoot){
				mSwerve.driveAndFaceAngle(swerveTranslation, SwerveConstants.robotForwardAngle, robotCentric);
			} else if(prepareShooterForShot){
				mSwerve.driveAndFaceAngle(swerveTranslation, SwerveConstants.robotForwardAngle, robotCentric);
			} else if(mControlBoard.getLockWheels()){
				mSwerve.lockWheels();
			} else if(mControlBoard.getDodgeCounterClockwise()){
				mSwerve.driveAndDodge(swerveTranslation, DodgeDirection.Counterclockwise);
			} else if(mControlBoard.getDodgeClockwise()){
				mSwerve.driveAndDodge(swerveTranslation, DodgeDirection.Clockwise);
			} else if(normalDriveRequested){
				mSwerve.drive(swerveTranslation, swerveRotation.getRadians(), robotCentric);
			} else if(mTimerSinceSwerveEnabled.get() >= SwerveConstants.disabledTimeBeforeLockWheels){
				mSwerve.lockWheels();
			} else{
				mSwerve.disable();
			}


			//arm manual setpoint tweaking(for if the zero is wrong)
			if(mControlBoard.getTweakArmSetpointUp()){
				mArm.incrementSensorReading(ControlBoardConstants.kTweakArmSetpointAmount);
			} else if(mControlBoard.getTweakArmSetpointDown()){
				mArm.incrementSensorReading(ControlBoardConstants.kTweakArmSetpointAmount.times(-1));
			}
			//arm rezero buttons
			if(mControlBoard.getManualControllerResetPivot()){
				mArm.resetPivotToAngle(Rotation2d.fromDegrees(-90));
			}
			if(mControlBoard.getZeroTelescope()){
				mArm.zeroTelescope();
			}


			/////////  put all of the manual override code here  //////////
			final boolean armManualOverride = mControlBoard.getArmManualOverride();

			if(armManualOverride){
				double pivotPO = mControlBoard.getPivotPO();
				double telescopePO = mControlBoard.getTelescopePO();
				double multiplier = (mControlBoard.getArmManualSlowMode()? 0.5 : 1);
				mArm.setPercentOutput(pivotPO*multiplier, telescopePO*multiplier);
			}


		}


		if(mAutoRoutineExecutor.isFinished()){
			mSelectedRoutine = Optional.empty();
		}
	}


	private boolean shouldSelectNewAutoRoutine(Optional<ActionBase> desiredAutoRoutine) {
		return !mSelectedRoutine.toString().equals(desiredAutoRoutine.toString());
	}
	
	
	private void selectNewAutoRoutine(Optional<ActionBase> newRoutine){
		mSelectedRoutine = newRoutine;
		System.out.println("new auto routine");
		mAutoRoutineExecutor.stop();
		if(newRoutine.isPresent()){
			mAutoRoutineExecutor.selectRoutine(mSelectedRoutine.get());
			mAutoRoutineExecutor.start();
		}
	}
}