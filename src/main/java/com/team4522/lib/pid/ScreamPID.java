package com.team4522.lib.pid;

import edu.wpi.first.math.MathUtil;

public class ScreamPID {

	private PIDConstants mConstants;
  
	private double mMaximumInput;
	private double mMinimumInput;
	private boolean mContinuous = false;
  
	private double mError = Double.NaN;
	private double mPrevError = Double.NaN;
	private double mIntegral = 0;
  
	private double mTolerance = 0;
  
	private double mSetpoint;
	private double mMeasurement;
  
	public ScreamPID(PIDConstants constants) {
		mConstants = constants.clone();
	}
	
	public void setPID(PIDConstants constants) {
	  	mConstants = constants.clone();
	}
	
	public void setSetpoint(double setpoint) {
		mSetpoint = setpoint;
	}
	
	public double getSetpoint() {
		return mSetpoint;
	}

	public boolean atSetpoint() {
		double positionError;
		if (mContinuous) {
			double errorBound = (mMaximumInput - mMinimumInput) / 2.0;
			positionError = MathUtil.inputModulus(mSetpoint - mMeasurement, -errorBound, errorBound);
		} else {
			positionError = mSetpoint - mMeasurement;
		}

		return Math.abs(positionError) < mTolerance;
	}

	public void enableContinuousInput(double minimumInput, double maximumInput) {
		mContinuous = true;
		mMinimumInput = minimumInput;
		mMaximumInput = maximumInput;
	}

	public void disableContinuousInput() {
		mContinuous = false;
	}

	public boolean isContinuousInputEnabled() {
		return mContinuous;
	}

	public void setTolerance(double tolerance) {
		mTolerance = tolerance;
	}

	public double getError() {
		return mError;
	}

	public double calculate(double measurement, double setpoint) {
		setSetpoint(setpoint);
		return calculate(measurement);
	}
	
	public double calculate(double measurement) {
	  mMeasurement = measurement;
	  mPrevError = mError;
  
	  if (mContinuous) {
		double errorBound = (mMaximumInput - mMinimumInput) / 2.0;
		mError = MathUtil.inputModulus(mSetpoint - mMeasurement, -errorBound, errorBound);
	  } else {
		mError = mSetpoint - measurement;
	  }
  
	  double derivative = Double.isNaN(mPrevError) ? 0 : (mError - mPrevError) / mConstants.period();
  
	  if (mConstants.kI() != 0) {
		mIntegral = MathUtil.clamp(	mIntegral + mError * mConstants.period(),
									mConstants.minIntegralAccumulator() / mConstants.kI(),
									mConstants.maxIntegralAccumulator() / mConstants.kI());
	  }
  
	  return mConstants.kP() * mError + mConstants.kI() * mIntegral + mConstants.kD() * derivative;
	}
  
	public void reset() {
		mError = Double.NaN;
		mPrevError = Double.NaN;
		mIntegral = 0;
	}
}
