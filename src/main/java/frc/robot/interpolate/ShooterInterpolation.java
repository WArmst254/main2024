package frc.robot.interpolate;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.InterpolatingDouble;
import frc.robot.Constants;
import frc.robot.Constants.Setpoints;

public class ShooterInterpolation {

    public static double calculateShooterRPM(double targetDistance) {

        return (MathUtil.clamp(Constants.kRPMMap.getInterpolated(new InterpolatingDouble(targetDistance)).value,
    Setpoints.shooterMinClamp, Setpoints.shooterMaxClamp));
}


     public static Rotation2d calculatePivotAngle(double targetDistance) {

            return Rotation2d.fromDegrees(MathUtil.clamp(Constants.kPivotMap.getInterpolated(new InterpolatingDouble(targetDistance)).value,
        Setpoints.pivotMinClamp, Setpoints.pivotMaxClamp));
    }
}
