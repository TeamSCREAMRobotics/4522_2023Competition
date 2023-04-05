package frc2023.battery;

import java.util.Arrays;
import java.util.List;

public class BatteryTask implements Comparable<BatteryTask>{
	private List<BatteryManageable> mManageables;
	private int mPriority;
	private boolean mForce;

	public BatteryTask(boolean force, int priority, List<BatteryManageable> manageables){
		mForce = force;
		mPriority = priority;
		mManageables = manageables;
	}

	public BatteryTask(boolean force, int priority, BatteryManageable ... manageables){
		mForce = force;
		mPriority = priority;
		mManageables = Arrays.asList(manageables);
	}

	public int priority(){
		return mPriority;
	}

	public List<BatteryManageable> manageables(){
		return mManageables;
	}

	public boolean force(){
		return mForce;
	}

	@Override
	public int compareTo(BatteryTask task) {
		if(task == null) throw new NullPointerException();

		if(force() == task.force()) return priority() - task.priority();
		if(force()) return Integer.MAX_VALUE;
		return Integer.MIN_VALUE;
	}
}
