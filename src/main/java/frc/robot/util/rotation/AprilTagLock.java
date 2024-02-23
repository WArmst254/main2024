package frc.robot.util.rotation;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AprilTagLock implements RotationSource {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    public static PIDController rotationPID = createPIDController();
    private static PIDController createPIDController() {
        PIDController pid = new PIDController(.01, .02, .001); //TODO: Tune
        pid.setTolerance(.25); //TODO: Tune
        pid.enableContinuousInput(0, 360);
        pid.setSetpoint(0); //TODO: Change? 0 = apriltag angle
        return pid;
    }
    @Override
    public double getR() {
        return rotationPID.calculate(table.getEntry("tx").getDouble(0)); //TODO: May need negative sign
    }
}
