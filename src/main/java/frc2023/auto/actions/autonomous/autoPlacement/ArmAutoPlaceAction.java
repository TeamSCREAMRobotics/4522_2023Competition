package frc2023.auto.actions.autonomous.autoPlacement;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc2023.auto.actions.lib.ActionBase;
import frc2023.controlboard.ControlBoard;
import frc2023.subsystems.Arm;
import frc2023.subsystems.Gripper;
import frc2023.subsystems.Gripper.GripperState;

public class ArmAutoPlaceAction extends ActionBase{

    private final Translation2d mPlaceLocation;
    private final Arm mArm = Arm.getInstance();
    private final Gripper mGripper = Gripper.getInstance();
    private final ControlBoard mControlBoard = ControlBoard.getInstance();
    private final Timer mTimerWaitAfterPlace = new Timer();
    public static final double timeAfterPlaceToFinish = 0.6;
    public static final double timeoutSeconds = 2.0;
    private final Timer mTimerSinceStart = new Timer();
    private final Timer mWaitAfterAtPositionTimer = new Timer();
    public static final double timeAfterAtPosition = .4;

    /**
     * This Action assumes that the robot is in the correct position and autoplaces accordingly. It is used for the beginning of autos when we know the swerve is in the right position and we auto place with arm only.
     * @param placeLocation
     */
    public ArmAutoPlaceAction(Translation2d placeLocation){
        mPlaceLocation = placeLocation;
    }

    @Override
    public void start() {
        System.out.println("arm auto place action");
        mTimerSinceStart.reset();
        mTimerSinceStart.start();
        mGripper.close();
    }

    @Override
    public void run() {
        mArm.setPositionWithoutLimits(mPlaceLocation, true);    
        if(mArm.atTargetPosition(mPlaceLocation) || mTimerSinceStart.get() >= timeoutSeconds){
            mWaitAfterAtPositionTimer.reset();
            mWaitAfterAtPositionTimer.start();
            mGripper.open();
        }

        if(mTimerSinceStart.get() >= .15) mGripper.open();
    }

    @Override
    public void stop(boolean interrupted) {
        System.out.println("arm auto place stop");
        mArm.disable();
        mGripper.open();
    }


    @Override
    public boolean isFinished() {
        if(mGripper.getLastState() == GripperState.OPEN){
            mTimerWaitAfterPlace.start();
        }
        
        return mTimerWaitAfterPlace.get() >= timeAfterPlaceToFinish || mTimerSinceStart.get() > timeoutSeconds + timeAfterPlaceToFinish || mControlBoard.getArmManualOverride();
    }

    @Override
    public String getID() {
        return "ArmAutoPlaceAction: " + mPlaceLocation.toString();
    }
}
