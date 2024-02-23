// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Vision extends SubsystemBase {

    public static NetworkTable table;

    public static void init() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    @Override
    public void periodic() {
        table.getEntry("pipeline").setDouble(1);
    }

    public double getDistance() {
        int tid = (int) table.getEntry("tid").getDouble(-1);
        int ty = (int) table.getEntry("ty").getDouble(0);
        if (tid == -1) return 0;
        double h2 = Constants.AprilTagHeights[tid - 1];
        double angleToGoal = Units.degreesToRadians(37 + ty); //TODO: mount pitch 37deg change
        double heightToGoal = h2 - 7.75; //TODO: mount height 7.75in change
        double distance = heightToGoal / Math.tan(angleToGoal);
        return Units.inchesToMeters(distance / 2);
    }

    public static boolean isSpeaker() {
        int tid = (int) table.getEntry("tid").getDouble(-1);
        return tid == 3 || tid == 4 || tid == 7 || tid == 8;
    }

}