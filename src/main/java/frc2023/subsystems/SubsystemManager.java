
package frc2023.subsystems;

import java.util.Arrays;
import java.util.List;

public class SubsystemManager {
	private static SubsystemManager mInstance = null;

    private List<Subsystem> mAllSubsystems;
    private SubsystemManager() {}

    public static SubsystemManager getInstance() {
        if (mInstance == null) {
            mInstance = new SubsystemManager();
        }
        return mInstance;
    }

    public void setSubsystems(Subsystem... allSubsystems) {
        mAllSubsystems = Arrays.asList(allSubsystems);
    }
    
	public void writeOutputs(){
		mAllSubsystems.forEach(Subsystem::writeOutputs);
	}

    public void outputTelemetry() {
        mAllSubsystems.forEach(Subsystem::outputTelemetry);
    }

    public void stopAllSubsystems() {
        mAllSubsystems.forEach(Subsystem::stop);
    }

    public void disableAllSubsystems() {
        mAllSubsystems.forEach(Subsystem::disable);
    }

    public List<Subsystem> getSubsystems() {
        return mAllSubsystems;
    }
}