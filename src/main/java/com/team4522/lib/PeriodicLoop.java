package com.team4522.lib;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;

public class PeriodicLoop {
    public final double mPeriod;

    private boolean mRunning;

    private final Notifier mNotifier;
    private final Object mTaskRunningLock = new Object();
    private double mTimestamp = 0;

    private final Runnable mPeriodicLoop;

    private final Runnable runnable_ = new Runnable() {
        @Override
        public void run() {
            synchronized (mTaskRunningLock) {
                if (mRunning) {
                    double now = Timer.getFPGATimestamp();

                    mPeriodicLoop.run();

                    mTimestamp = now;
                }
            }
        }
    };

    public PeriodicLoop(Runnable periodicLoop, double period) {

        mPeriodicLoop = periodicLoop;
        mNotifier = new Notifier(runnable_);
        mPeriod = period;
        mRunning = false;
    }

    public synchronized void start() {
        if (!mRunning) {
            System.out.println("Starting loop");

            synchronized (mTaskRunningLock) {
                mTimestamp = Timer.getFPGATimestamp();
                
                mRunning = true;
            }

            mNotifier.startPeriodic(mPeriod);
        }
    }

    public synchronized void stop() {
        if (mRunning) {
            System.out.println("Stopping loop");
            mNotifier.stop();

            synchronized (mTaskRunningLock) {
                mRunning = false;
                mTimestamp = Timer.getFPGATimestamp();
                
            }
        }
    }
    
    public double getTimestamp() {
        return mTimestamp;
    }
}