package frc2023.battery;

public abstract class BatteryManageable {

	public double getMinCurrent() {
		return 0.0;
	}

	public double allotAdditionalCurrent(double available){
		return available;
	}

	public void insufficientCurrent() {}
}
