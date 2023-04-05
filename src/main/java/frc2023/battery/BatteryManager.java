package frc2023.battery;

import java.util.List;

/**
 * In the 2022 Rapid React season, we had a lot of battery issues and our robot would brown out before endgame in almost every match. In the 2022 offseason, we came up
 * 		with a battery management system, where in the code we would define seperatable tasks that robot would perform, give them a priority list, and 'budget' the current
 * 		draw of the battery throughout robot usage. If the requested tasks were expected to take more current than the battery could handle, we would disable reduce the current
 * 		of tasks to the minimum current it would take for them to still function, and if needed, disable low-priority tasks. <p>
 * 
 * We never had the time to test the system and haven't needed to use it. We have much better electrical management of our batteries now, and the 2023 Charged Up game is not
 * 		very battery intensive.
 */
public class BatteryManager {

	private static BatteryManager mInstance = null;
	public static BatteryManager getInstance(){
		if(mInstance == null){
			mInstance = new BatteryManager();
		}
		return mInstance;
	}

	private BatteryManager(){}

	public double getAvailableCurrent(){
		// TODO implement. Right now we assume 180 amps.
		return 180.0;
	}

	/**
	 * Takes a list of BatteryTasks and determines which ones the robot can perform and disables any that the robot doesn't have enough current to accomplish. 
	 * 		If a BatteryTask cannot be performed, each subsystem doing the task disables.<p>
	 *  
	 * The algorithm we use gives current to each subsystem equal to the minimum current it takes to funciton. 
	 * 		If there is extra current after going through all of the tasks, we go back through the tasks and give them current equal to their initial demands
	 * @param tasks
	 */
	public void manage(List<BatteryTask> tasks){
		double available = getAvailableCurrent();

		tasks.sort(null);
		for(BatteryTask t : tasks){
			double required = 0.0;
			for(BatteryManageable bm : t.manageables()){//the required current for a battery task is the sum of all the predicted current draw of each subsystem doing the task
				required += bm.getMinCurrent();
			}

			if(t.force() || required <= available){//some battery tasks have a force parameter which means that the robot will do them no matter what. These tasks are calculated first.
				available -= required;
			}else{//if a task cannot be accomplished, then all of its subsystems call isufficientCurrent, which does the logic for disabling the subsystem
				t.manageables().forEach(BatteryManageable::insufficientCurrent);
				tasks.remove(t);
			}
		}

		for(BatteryTask t : tasks){//after calculating whether each subsystems can run at their minimum desired current, we allot additional current to each subsystem to meet the the demands
			for(BatteryManageable bm : t.manageables()){
				available = bm.allotAdditionalCurrent(available);
			}
		}
	}

	public double getCurrentDraw(){
		return 0; //TODO
	}

	public double getBatteryVoltage(){
		return 0; //TODO
	}

	public double getPowerConsumption(){
		return 0; //TODO
	}
}