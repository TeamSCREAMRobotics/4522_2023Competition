package com.team4522.lib.drivers;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class ScreamXboxController {

	private final XboxController mController;

	public ScreamXboxController(int port){
		mController = new XboxController(port);
	}

	public enum Button {
        A(1), B(2), X(3), Y(4), LB(5), RB(6), BACK(7), START(8), L_JOYSTICK(9), R_JOYSTICK(10);

        public final int id;

        Button(int id) {
            this.id = id;
        }
    }

    public enum POV {
        CENTER(-1), UP(0), UP_RIGHT(45), RIGHT(90), DOWN_RIGHT(135), DOWN(180), DOWN_LEFT(225), LEFT(270), UP_LEFT(315);
        
        public final int angle;

        POV(int angle){
            this.angle = angle;
        }
    }

	public enum Side{
		LEFT, RIGHT
	}

	public enum Axis{
		X, Y
	}

	public double getAxis(Side side, Axis axis) {
        boolean left = side == Side.LEFT;
        boolean y = axis == Axis.Y;
        return mController.getRawAxis((left ? 0 : 4) + (y ? 1 : 0));
    }

    public double getTrigger(Side side) {
        return mController.getRawAxis(side == Side.LEFT ? 2 : 3);
    }

    public boolean getButton(Button button) {
        return mController.getRawButton(button.id);
    }

    public boolean getButtonPressed(Button button) {
        return mController.getRawButtonPressed(button.id);
    }

    public boolean getButtonReleased(Button button) {
        return mController.getRawButtonReleased(button.id);
    }

    /**
     * @param rumbleType Left, Right, or Both sides rumbling
     * @param intensity The value for how intense the rumble is. Value ranges from 0 to 1
     */
    public void setRumble(RumbleType rumbleType, double intensity) {
        mController.setRumble(rumbleType, intensity);
    }

    public POV getPOV(){
        switch(mController.getPOV()){
            case 0: return POV.UP;
            case 45: return POV.UP_RIGHT;
            case 90: return POV.RIGHT;
            case 135: return POV.DOWN_RIGHT;
            case 180: return POV.DOWN;
            case 225: return POV.DOWN_LEFT;
            case 270: return POV.LEFT;
            case 315: return POV.UP_LEFT;
            default: return POV.CENTER;
        }
    }
}
