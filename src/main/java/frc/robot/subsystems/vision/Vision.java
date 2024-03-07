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
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    public static void init() {
    }

    @Override
    public void periodic() {
    }

    public double getDistance() {
        double ty = table.getEntry("ty").getDouble(0);
        double tid =  table.getEntry("tid").getDouble(-1);
       if (tid == -1) return 0;
        double h2 = Constants.AprilTagHeights[1];
        double angleToGoal = Units.degreesToRadians(13 + ty);
        double heightToGoal = h2 - 26;
        double distance = heightToGoal / Math.tan(angleToGoal);
        return Units.inchesToMeters(distance);
    }

    public boolean isValidTarget() {
        return(NetworkTableInstance.getDefault().getEntry("tv").getDouble(0) == 1);

    }

}