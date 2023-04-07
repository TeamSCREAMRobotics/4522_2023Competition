package frc2023.shuffleboard;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public abstract class ShuffleboardTabBase {
    protected ShuffleboardTab mTab;

    public abstract void createEntries();

    protected GenericEntry createNumberEntry(String name, double defaultValue) {//We can create entries of all the object types with the createEntry() method, but we 
                                                                                //have different methods to be explicit about the type of value we want on Shuffleboard   
        return mTab.add(name, defaultValue).withSize(1, 1).getEntry();
    }

    protected GenericEntry createBooleanEntry(String name, boolean defaultValue) {
        return mTab.add(name, defaultValue).withSize(1, 1).getEntry();
    }

    protected GenericEntry createStringEntry(String name, String defaultValue) {
        return mTab.add(name, defaultValue).withSize(1, 1).getEntry();
    }

    protected GenericEntry createEntry(String name, Object defaultValue) {
        return mTab.add(name, defaultValue).withSize(1, 1).getEntry();
    }

    public abstract void update();

    protected double round(double number, int decimalPlaces) {
        double tmp = Math.pow(10, decimalPlaces);
        return Math.round(number * tmp) / tmp;
    }
    
    public ShuffleboardTab getTab() {
        return mTab;
    }
}