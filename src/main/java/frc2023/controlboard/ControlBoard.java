package frc2023.controlboard;

import java.util.Optional;

import com.team4522.lib.drivers.ScreamXboxController;
import com.team4522.lib.drivers.ScreamXboxController.Axis;
import com.team4522.lib.drivers.ScreamXboxController.Button;
import com.team4522.lib.drivers.ScreamXboxController.Side;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc2023.Constants.*;
import frc2023.PlacementStates.Level;
import frc2023.PlacementStates.Node;
import frc2023.controlboard.Buttonboard.Direction;
import frc2023.Ports;

public class ControlBoard {

	private final ScreamXboxController mController;
	private final Buttonboard mButtonboard;
	private final ScreamXboxController mManualController;

	private ControlBoard(){
		mController = new ScreamXboxController(Ports.driverControllerPort);
		mButtonboard = new Buttonboard(Ports.buttonboardPortA, Ports.buttonboardPortB);
		mManualController = new ScreamXboxController(3);
	}
	
	private static ControlBoard mInstance = null;
	public static ControlBoard getInstance(){
		if(mInstance == null){
			mInstance = new ControlBoard();
		}
		return mInstance;
	}

	public Translation2d getSwerveTranslation(){
		Translation2d input = new Translation2d(mController.getAxis(Side.LEFT, Axis.X),
												-mController.getAxis(Side.LEFT, Axis.Y));

		Translation2d deadbandVector = input.times(ControlBoardConstants.kSwerveTranslationDeadband/input.getNorm());
		double x = Math.signum(input.getX()) * MathUtil.applyDeadband(Math.abs(input.getX()), Math.abs(deadbandVector.getX()));
		double y =  Math.signum(input.getY()) * MathUtil.applyDeadband(Math.abs(input.getY()), Math.abs(deadbandVector.getY()));

		Translation2d deadbandedInput = new Translation2d(x, y);
		deadbandedInput = snapToPole(deadbandedInput);

		return deadbandedInput.times(Math.pow(deadbandedInput.getNorm(), ControlBoardConstants.kTranslationJoystickSmoothingExponent)).times(SwerveConstants.kMaxDriveSpeed);
	}

	private Translation2d snapToPole(Translation2d driveInput){
		
		Rotation2d threshold = ControlBoardConstants.kThresholdToSnapSwerveToPole;
		for(int i = 0; i < 360; i+=90){

			if(Math.abs(driveInput.getAngle().minus(Rotation2d.fromDegrees(i)).getDegrees()) < threshold.getDegrees()) return new Translation2d(driveInput.getNorm(), Rotation2d.fromDegrees(i));
		}
		return driveInput;
	}

	public Rotation2d getSwerveRotation(){
		double turn = mController.getAxis(Side.RIGHT, Axis.X);
		double rotationInput = -Math.signum(turn) * MathUtil.applyDeadband(Math.abs(turn), ControlBoardConstants.kSwerveRotationDeadband, 1);
		double deadbandedAndSquaredRotation = Math.signum(rotationInput) * Math.pow(Math.abs(rotationInput), ControlBoardConstants.kRotationJoystickSmoothingExponent);

		return Rotation2d.fromRadians(deadbandedAndSquaredRotation*SwerveConstants.kMaxDriveAngularSpeed);
	}

	public Optional<Rotation2d> getSwerveTargetAngle(){
		switch(mController.getPOV()){
			case LEFT: return Optional.of(Rotation2d.fromDegrees(180));
			// case UP_LEFT: return Optional.of(Rotation2d.fromDegrees(135));
			case UP: return Optional.of(Rotation2d.fromDegrees(90));
			// case UP_RIGHT: return Optional.of(Rotation2d.fromDegrees(45));
			case RIGHT: return Optional.of(Rotation2d.fromDegrees(0));
			// case DOWN_RIGHT: return Optional.of(Rotation2d.fromDegrees(-45));
			case DOWN: return Optional.of(Rotation2d.fromDegrees(-90));
			// case DOWN_LEFT: return Optional.of(Rotation2d.fromDegrees(-135));
			case CENTER:
			default: return Optional.empty();
		}
		
	}
	

	Node mSelectedNode = Node.NODE1;
	private void updateSelectedNode(){
		Alliance alliance = DriverStation.getAlliance();
		if(alliance == Alliance.Blue){
			if(mButtonboard.getRawButton(1)) mSelectedNode = Node.NODE1;
			else if(mButtonboard.getRawButton(2)) mSelectedNode = Node.NODE2;
			else if(mButtonboard.getRawButton(3)) mSelectedNode = Node.NODE3;
			else if(mButtonboard.getRawButton(4)) mSelectedNode = Node.NODE4;
			else if(mButtonboard.getRawButton(5)) mSelectedNode = Node.NODE5;
			else if(mButtonboard.getRawButton(6)) mSelectedNode = Node.NODE6;
			else if(mButtonboard.getRawButton(7)) mSelectedNode = Node.NODE7;
			else if(mButtonboard.getRawButton(8)) mSelectedNode = Node.NODE8;
			else if(mButtonboard.getRawButton(9)) mSelectedNode = Node.NODE9;
		} else{
			
			if(mButtonboard.getRawButton(9)) mSelectedNode = Node.NODE1;
			else if(mButtonboard.getRawButton(8)) mSelectedNode = Node.NODE2;
			else if(mButtonboard.getRawButton(7)) mSelectedNode = Node.NODE3;
			else if(mButtonboard.getRawButton(6)) mSelectedNode = Node.NODE4;
			else if(mButtonboard.getRawButton(5)) mSelectedNode = Node.NODE5;
			else if(mButtonboard.getRawButton(4)) mSelectedNode = Node.NODE6;
			else if(mButtonboard.getRawButton(3)) mSelectedNode = Node.NODE7;
			else if(mButtonboard.getRawButton(2)) mSelectedNode = Node.NODE8;
			else if(mButtonboard.getRawButton(1)) mSelectedNode = Node.NODE9;
		}
		
	}

	Level mSelectedLevel = Level.TOP;
	private void updateSelectedPlacementLevel(){
		if(mButtonboard.getRawButton(12)) mSelectedLevel = Level.TOP;
		else if(mButtonboard.getRawButton(11)) mSelectedLevel = Level.MIDDLE;
		else if(mButtonboard.getRawButton(10)) mSelectedLevel =Level.HYBRID;
	}

	public boolean getAutoPlaceWithoutPosition(){
		return mController.getButton(Button.A);
	}

	public boolean getSlowMode(){
		return mController.getTrigger(Side.LEFT) > ControlBoardConstants.kTriggerThreshold;
	}

	public boolean getBackwardsEject(){
		return mController.getButton(Button.X);
	}

	public boolean getPrepareIntakeForShot(){
		return mController.getButton(Button.Y);
	}

	public boolean getLockWheels(){
		return false;//mController.getButton(Button.B);
	}	
	
	public boolean getZeroGyro(){
		return mController.getButton(Button.BACK);
	}
	
	public boolean getZeroPose(){
		return mController.getButton(Button.START);
	}

	public boolean getIntake(){
		return mController.getTrigger(Side.RIGHT) > ControlBoardConstants.kTriggerThreshold;
	}

	public boolean getEject(){
		return mController.getButton(Button.LB);
	}

	public boolean getRobotCentric(){
		return false;
	}

	public boolean getManualControllerResetPivot(){
		return mManualController.getButton(Button.START);
	}

	public boolean getDodgeCounterClockwise() {
		return false;//mController.getButton(Button.L_JOYSTICK);
	}

    public boolean getDodgeClockwise() {
        return false; //mController.getButton(Button.R_JOYSTICK);
    }

    public boolean getDriveAndFaceAngle() {
        return false;
    }

	public void setRumbleForEndGame(){
		mController.setRumble(RumbleType.kBothRumble, 0.3);
	}

	public void update(){
		updateSelectedNode();
		updateSelectedPlacementLevel();
	}


	public boolean getGripperOpenForConeIntake(){
		return mButtonboard.getBigSwitch() == Direction.DOWN;
	}

	public boolean getGripperOpenForPlace(){
		return getGripperOpenForConeIntake();//they are the same, but this is easier to read
	}

	public boolean getCloseGripper(){
		return mButtonboard.getBigSwitch() == Direction.UP;
	}

	public boolean getGripperFullOpen(){
		return mButtonboard.getBigSwitch() == Direction.RIGHT;
	}

	public boolean armPlaceCone(){
		return mButtonboard.getRawSwitch(2);
	}

	public boolean getArmManualOverride(){
		return getManualOverrideSwtich();
	}

	private boolean getManualOverrideSwtich(){
		return mButtonboard.getRawSwitch(3);
	}
	public double getPivotPO(){
		double input = mManualController.getAxis(Side.LEFT, Axis.Y);
		double y =  Math.signum(input) * MathUtil.applyDeadband(Math.abs(input), Math.abs(ControlBoardConstants.manualPivotPODeadband));
		
		return Math.signum(y) * Math.pow(y, 2) / 2;
	}

	public double getTelescopePO(){
		double input = mManualController.getAxis(Side.RIGHT, Axis.Y);
		double y =  Math.signum(input) * MathUtil.applyDeadband(Math.abs(input), Math.abs(ControlBoardConstants.manualPivotPODeadband));
		
		return Math.signum(y) * Math.pow(y, 2) / 1.0;
	}
	
	public boolean getShootCube(){
		return mController.getButton(Button.RB);
	}

	public boolean getAutoShoot(){
		return false;//mController.getButton(Button.R_JOYSTICK);
	}

	public boolean getSweep(){
		return mController.getButton(Button.L_JOYSTICK);
	}

	public Node getSelectedNode() {
		return mSelectedNode;
	}

	public Level getSelectedLevel() {
		return mSelectedLevel;
	}

	public boolean getZeroTelescope(){
		return mManualController.getButton(Button.BACK);
	}

	public boolean getTweakArmSetpointUp(){
		return mManualController.getButtonPressed(Button.Y);
	}

	
	public boolean getTweakArmSetpointDown(){
		return mManualController.getButtonPressed(Button.A);
	}

	
	public boolean getArmManualSlowMode(){
		return mManualController.getTrigger(Side.LEFT) >= ControlBoardConstants.kTriggerThreshold;
	}

    public boolean getSnapAngleAndShoot() {
        return mController.getButton(Button.B);
    }

    public boolean getPoopShootFromChargeLine() {
        return mController.getButton(Button.R_JOYSTICK);
    }

    public boolean getArmHoldSetpoint() {
        return mButtonboard.getRawSwitch(1);
    }

    public boolean getPreparePlacement() {
        return false;//Buttonboard.getRawSwitch(4);
    }

	public double getSlowModeTranslationScalar() {
		return MathUtil.interpolate(ControlBoardConstants.kSlowModeTranslationMinScalar, 1, 1-mController.getTrigger(Side.LEFT));
	}


	public double getSlowModeRotationScalar() {
		return	MathUtil.interpolate(ControlBoardConstants.kSlowModeRotationMinScalar, 1, 1-mController.getTrigger(Side.LEFT));		
	}

	public boolean getFullAutoPlace() {
		return false;
	}

}