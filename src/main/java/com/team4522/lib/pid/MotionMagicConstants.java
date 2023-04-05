package com.team4522.lib.pid;

public class MotionMagicConstants {
    public final double cruiseVelocity;//native units
    public final double acceleration;//native units
    public final int sCurveStrength;//bounded [0, 8]

    /**
     * @param cruiseVelocity native units
     * @param acceleration native units
     * @param sCurveStrength bounded [0, 8]
     */
    public MotionMagicConstants(double cruiseVelocity, double acceleration, int sCurveStrength){
        this.cruiseVelocity = cruiseVelocity;
        this.acceleration = acceleration;
        this.sCurveStrength = sCurveStrength;
    }
}
