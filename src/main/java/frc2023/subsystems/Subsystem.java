
package frc2023.subsystems;

public abstract class Subsystem{

	public abstract void stop();

	public abstract void writeOutputs();

	public void outputTelemetry() {}

	public abstract void disable();

	// public void registerLog(LogManager logManager) {}
}