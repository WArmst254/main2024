package frc.robot.util.rotation;

import frc.robot.RobotContainer;

public class Joystick implements RotationSource {
    @Override
    public double getR() {
        return -RobotContainer.driverController.getRightX();
    }
}