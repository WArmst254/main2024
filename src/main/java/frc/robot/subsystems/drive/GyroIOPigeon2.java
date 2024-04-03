package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Pigeon2Configurator;
import com.ctre.phoenix6.hardware.Pigeon2;

import frc.robot.util.NRUnits;

public class GyroIOPigeon2 implements GyroIO {
    
    private Pigeon2 gyro;
    private Pigeon2Configurator gyroConfigurator;
    private Pigeon2Configuration gyroConfig;

    public GyroIOPigeon2() {
        gyro = new Pigeon2(0, "Canivore");
        gyroConfigurator = gyro.getConfigurator();
        config();

    }

    private void config() {
        gyroConfig = new Pigeon2Configuration();
        gyroConfig.MountPose.MountPoseYaw = -0.263672;
        gyroConfig.MountPose.MountPosePitch = 0.307617;
        gyroConfig.MountPose.MountPoseRoll = -0.483398;

        gyroConfig.Pigeon2Features.DisableNoMotionCalibration = false;
        gyroConfigurator.apply(gyroConfig);
    }

    public void updateInputs(GyroIOInputs inputs) {
        inputs.yaw = gyro.getYaw().getValueAsDouble() * (2*Math.PI)/360;
        inputs.constrainedYaw = NRUnits.constrainRad(gyro.getYaw().getValueAsDouble()*(2*Math.PI)/360);
        inputs.pitch = gyro.getPitch().getValue() * (2*Math.PI)/360;
        inputs.roll = gyro.getRoll().getValue() * (2*Math.PI)/360;

        inputs.xVelocity = gyro.getAngularVelocityXWorld().getValueAsDouble() * (2*Math.PI)/360;
        inputs.yVelocity = gyro.getAngularVelocityYWorld().getValueAsDouble() * (2*Math.PI)/360;
        inputs.zVelocity = gyro.getAngularVelocityZWorld().getValueAsDouble() * (2*Math.PI)/360;
    }

    public void setYaw(double angle) {
        gyro.setYaw(angle);
    }

}
