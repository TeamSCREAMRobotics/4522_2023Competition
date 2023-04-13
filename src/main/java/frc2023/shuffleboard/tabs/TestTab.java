package frc2023.shuffleboard.tabs;

import com.team4522.lib.pid.PIDConstants;

import edu.wpi.first.networktables.GenericSubscriber;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc2023.Constants.IntakeConstants;
import frc2023.Constants.SwerveConstants;
import frc2023.Constants.IntakeConstants.LowerConveyorConstants;
import frc2023.Constants.IntakeConstants.UpperConveyorConstants;
import frc2023.shuffleboard.ShuffleboardTabBase;
import frc2023.subsystems.Swerve;

public class TestTab extends ShuffleboardTabBase{

    public GenericSubscriber shootingSpeedTop;
    public GenericSubscriber shootingSpeedBottom;
    public GenericSubscriber shootingSpeedRoller;
    public GenericSubscriber shootingSpeedPoopShooter;

    
    public GenericSubscriber rotationHelperKP;
    public GenericSubscriber rotationHelperKI;
    public GenericSubscriber rotationHelperIZone;
    public GenericSubscriber rotationHelperKD;

    public GenericSubscriber autoBalancekP;

	private static TestTab mInstance = null;
	public static TestTab getInstance(){
		if(mInstance == null){
			mInstance = new TestTab();
		}
		return mInstance;
	}

    public TestTab(){
        
    }

    @Override
    public void createEntries() {
		mTab = Shuffleboard.getTab("TestTab");

        shootingSpeedRoller = createNumberEntry("RollerSpeed", IntakeConstants.kShootHighPO);
        shootingSpeedTop = createNumberEntry("TopConvSpeed", UpperConveyorConstants.kShootHighPO);
        shootingSpeedBottom = createNumberEntry("BottomConvSpeed", LowerConveyorConstants.kShootHighPO);
        shootingSpeedPoopShooter = createNumberEntry("Shooter PO", 0.0);

        autoBalancekP = createEntry("Auto Balance kP", SwerveConstants.autoBalancePIDConstants.kP());

        
        rotationHelperKP = createNumberEntry("rotationHelperKP", SwerveConstants.snapRotationPIDConstants.kP());
        rotationHelperKI = createNumberEntry("rotationHelperKI", SwerveConstants.snapRotationPIDConstants.kI());
        rotationHelperIZone = createNumberEntry("rotationHelperIZone", SwerveConstants.snapRotationPIDConstants.integralZone());
        rotationHelperKD = createNumberEntry("rotationHelperKD", SwerveConstants.snapRotationPIDConstants.kD());
    }

    @Override
    public void update() {
        
    }

    public void updateAutoBalancePIDConstants() {
        SwerveConstants.autoBalancePIDConstants.setkP(autoBalancekP.getDouble(SwerveConstants.autoBalancePIDConstants.kP()));
    }
    public void updateRotationHelperPIDConstants(){
        PIDConstants constants = new PIDConstants(rotationHelperKP.getDouble(0), rotationHelperKI.getDouble(0), rotationHelperKD.getDouble(0));
        constants.setIntegralZone(rotationHelperIZone.getDouble(0));
        Swerve.getInstance().mSwerveDriveHelper.mRotationHelper.setSnapPID(constants);
    }
}