package frc.robot.subsystems.amp;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Amp extends SubsystemBase {
  private final TalonFX ampLeft = new TalonFX(Constants.IDs.ampleft);
  public final TimeOfFlight amp_sensor = new TimeOfFlight(Constants.IDs.ampsensor);

  public Amp() {
    setDefaultCommand(
        runOnce(
                () -> {
                  ampLeft.set(0);
                })
            .andThen(run(() -> {}))
            .withName("Amp Idle"));
  }

  public void ampOuttakeOn() {
    ampLeft.set(0.9);
  }

  public void ampIntakeOn() {
    ampLeft.set(-.9);
  }

  public void disableAmp() {
    ampLeft.set(0);
  }

  public boolean ampSensorOut() {
    return (amp_sensor.getRange() < 200);
  }

  public double ampSensor() {
    return amp_sensor.getRange();
  }

  public boolean invAmpSensorOut() {
    return !(amp_sensor.getRange() < 200);
  }

  public void periodic() {
    SmartDashboard.putNumber("Amp Left Power: ", ampLeft.get());
  }
}
