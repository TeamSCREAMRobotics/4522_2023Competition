package frc2023.swerve;

import java.util.LinkedList;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;

public class SwerveSpeedBuffer {
		private final LinkedList<Twist2d> robotBufferedSpeeds = new LinkedList<Twist2d>();//there is probably something that works better for this than a linkedList
        private Twist2d mLatestTwist = new Twist2d();

        private final int mBufferSize;
        public SwerveSpeedBuffer(int bufferSize){
            mBufferSize = bufferSize;
            for(int i = 0; i < mBufferSize; i++) robotBufferedSpeeds.add(new Twist2d());//initializes the linked list with empty twists
        }

        private double previousTimestamp = 0.0;
        private double dt = 0.0;
        
        public void updateMeasurement(Twist2d twist, double timestamp){
            
            dt = timestamp - previousTimestamp;

		    Twist2d speed = new Twist2d(twist.dx/dt, twist.dy/dt, twist.dtheta/dt);

            //swap out the oldest measurement with a new measurement
            robotBufferedSpeeds.addLast(speed);
            robotBufferedSpeeds.removeFirst();

            previousTimestamp = timestamp;
            mLatestTwist = calculateSpeeds();
        }

        private Twist2d calculateSpeeds(){
            double xSpeed = 0; double ySpeed = 0; double thetaSpeed = 0;
            
            for(int i = 0; i < mBufferSize; i++){
                xSpeed += robotBufferedSpeeds.get(i).dx;
                ySpeed += robotBufferedSpeeds.get(i).dy;
                thetaSpeed += robotBufferedSpeeds.get(i).dtheta;
            }

            xSpeed /= mBufferSize;
            ySpeed /= mBufferSize;
            thetaSpeed /= mBufferSize;

            return new Twist2d(xSpeed, ySpeed, thetaSpeed);
        }

        public Twist2d getBufferedSpeeds(){
            return mLatestTwist;
        }

        
        public Translation2d getBufferedTranslationalSpeed(){
            return new Translation2d(mLatestTwist.dx, mLatestTwist.dy);
        }

        
        public Rotation2d getBufferedRotationalSpeed(){
            return Rotation2d.fromRadians(mLatestTwist.dtheta);
        }
}