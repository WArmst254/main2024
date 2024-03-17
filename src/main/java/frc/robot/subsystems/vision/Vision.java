// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {
    
    public static NetworkTable getAprilTagDetector(){
        return NetworkTableInstance.getDefault().getTable("limelight");
    }

    public static void init() {
    }

    @Override
    public void periodic() {
    }


    public double getDistance() {
        double ty = getAprilTagDetector().getEntry("ty").getDouble(0);
        double tid =  getAprilTagDetector().getEntry("tid").getDouble(-1);
       if (tid == -1) return 0;
        double h2 = Constants.AprilTagHeights[1];
        double angleToGoal = Units.degreesToRadians(24 + ty);
        double heightToGoal = h2 - 20.5;
        double distance = heightToGoal / Math.tan(angleToGoal);
        return Units.inchesToMeters(distance);
    }

    public static boolean canSeeAprilTag(){
        return getAprilTagDetector().getEntry("tv").getDouble(0) == 1;
    }

    public static double[] getBotPoseArray(){
        return getAprilTagDetector().getEntry("botpose_wpiblue").getDoubleArray(new double[6]);
    }

    public static Pose2d getBotPose(){
        double[] poseArray = getBotPoseArray();
        return new Pose2d(poseArray[0],poseArray[1],Rotation2d.fromDegrees(poseArray[5]));
    }

    public static double getLatency(){
        return Timer.getFPGATimestamp() - getBotPoseArray()[6]/1000.0;
    }

}