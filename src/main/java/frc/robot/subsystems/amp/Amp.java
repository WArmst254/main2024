package frc.robot.subsystems.amp;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Amp extends SubsystemBase {
  private final TalonFX ampLeft = new TalonFX(18);
  private final TalonFX ampRight = new TalonFX(19);
  public final TimeOfFlight amp_sensor = new TimeOfFlight(3);

  public Amp() {
    setDefaultCommand(
        runOnce(
                () -> {
                  ampLeft.disable();
                  ampRight.disable();
                })
            .andThen(run(() -> {}))
            .withName("Amp Idle"));
  }

  public void AmpOuttakeOn() {
    ampLeft.set(.95);
    ampRight.set(.95);
  }

  public void AmpOuttakeOff() {
    ampLeft.set(0);
    ampRight.set(0);
  }

  public boolean ampSensorOut() {
    return (amp_sensor.getRange() < 200);
  }

  public boolean invAmpSensorOut() {
    return !(amp_sensor.getRange() < 200);
  }

  public Command disableAmp() {
    return runOnce(
            () -> {
              ampLeft.disable();
              ampRight.disable();
            })
        .andThen(run(() -> {}).withTimeout(0.01))
        .withName("Amp Idle");
  }
}
