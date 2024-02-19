package frc.robot.subsystems.amp;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// elevator up
// amp outtake

// intakeamp until sensor 3 is hit
// ampouttake
public class Amp extends SubsystemBase {
  public final TimeOfFlight amp_Sensor = new TimeOfFlight(3);
  private final TalonFX AmpLeft = new TalonFX(18);
  private final TalonFX AmpRight = new TalonFX(19);

  public void AmpOuttakeOn() {
    AmpLeft.set(.95);
    AmpRight.set(.95);
  }

  public void AmpOuttakeOff() {
    AmpLeft.set(0);
    AmpRight.set(0);
  }

  public boolean ampSensorOut() {
    return (amp_Sensor.getRange() < 200);
  }

  public boolean invampSensorOut() {
    return !(amp_Sensor.getRange() < 200);
  }

  public Command disableAmp() {
    return runOnce(
            () -> {
              AmpLeft.disable();
              AmpRight.disable();
            })
        .andThen(run(() -> {}).withTimeout(0.05))
        .withName("Idle");
  }
}
