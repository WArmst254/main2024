package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final TalonFX intake = new TalonFX(17);
  private final TalonFX frontFeed = new TalonFX(13);
  private final TalonFX backFeed = new TalonFX(14);
  public final TimeOfFlight intake_sensor = new TimeOfFlight(1);

  public Intake() {
    setDefaultCommand(
        runOnce(
                () -> {
                  intake.disable();
                  frontFeed.disable();
                  backFeed.disable();
                })
            .andThen(run(() -> {}))
            .withName("Intake Idle"));
  }
 
  public void intakeOnShoot() {
    intake.set(1);
    frontFeed.set(-0.95);
    backFeed.set(0.95);
  }

  public void intakeOnAmp() {
    intake.set(1);
    frontFeed.set(-0.95);
    backFeed.set(-0.95);
  }

  public void outakeOnShoot() {
    intake.set(-1);
    frontFeed.set(0.95);
    backFeed.set(-0.95);
  }

  public void outakeOnAmp() {
    intake.set(-1);
    frontFeed.set(0.95);
    backFeed.set(0.95);
  }

  public void intakeOn() {
    intake.set(1);
  }

  public void intakeOff() {
    intake.set(0);
    frontFeed.set(0);
    backFeed.set(0);
  }

  public boolean intakeSensorOut() {
    return (intake_sensor.getRange() < 300);
  }

  public boolean invIntakeSensorOut() {
    return !(intake_sensor.getRange() < 300);
  }

  public Command disableIntake() {
    return runOnce(
            () -> {
              intake.disable();
              frontFeed.disable();
              backFeed.disable();
            })
        .andThen(run(() -> {}).withTimeout(0.01))
        .withName("Intake Idle");
  }
}
