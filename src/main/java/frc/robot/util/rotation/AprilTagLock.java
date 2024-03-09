package frc.robot.util.rotation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AprilTagLock implements RotationSource {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    public static PIDController rotationPID = createPIDController();
    private static PIDController createPIDController() {
        PIDController pid = new PIDController(0.020, 0.00006, 0.00006);
        pid.setTolerance(0);
        pid.enableContinuousInput(0, 360);
        pid.setSetpoint(0);
        return pid;
    }
    @Override
    public double getR() {
        return rotationPID.calculate(table.getEntry("tx").getDouble(0));
    }
    
}
