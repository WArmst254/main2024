package frc.robot.subsystems.amp;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Amp extends SubsystemBase {
  private final TalonFX ampLeft = new TalonFX(18);
  private final TalonFX ampRight = new TalonFX(19);
  public final TimeOfFlight amp_sensor = new TimeOfFlight(3);

  public Amp() {
    setDefaultCommand(
        runOnce(
                () -> {
                  ampLeft.set(0);
                  ampRight.set(0);
                })
            .andThen(run(() -> {}))
            .withName("Amp Idle"));
  }

  public void ampOuttakeOn() {
    ampLeft.set(.95);
    ampRight.set(.95);
  }

  public void ampIntakeOn() {
    ampLeft.set(-.95);
    ampRight.set(-.95);
  }

  public void disableAmp() {
    ampLeft.set(0);
    ampRight.set(0);
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
    SmartDashboard.putNumber("Amp Right Power: ", ampRight.get());
  }
}
