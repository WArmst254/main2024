package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private final TalonFX intake = new TalonFX(17);
  private final TalonFX feedFront = new TalonFX(13);
  private final TalonFX feedBack = new TalonFX(14);
  public final TimeOfFlight front_Sensor = new TimeOfFlight(1);
  public final TimeOfFlight shoot_Sensor = new TimeOfFlight(2);

  public void intakeOnShoot() {
    intake.set(1);
    feedFront.set(-0.95);
    feedBack.set(0.95);
  }

  public void intakeOnAmp() {
    intake.set(1);
    feedFront.set(-0.95);
    feedBack.set(-0.95);
  }

  public void outakeOnShoot() {
    intake.set(-1);
    feedFront.set(0.95);
    feedBack.set(-0.95);
  }

  public void outakeOnAmp() {
    intake.set(-1);
    feedFront.set(0.95);
    feedBack.set(0.95);
  }

  public void intakeOn() {
    intake.set(1);
  }

  public void intakeOff() {
    intake.set(0);
    feedFront.set(0);
    feedBack.set(0);
  }

  public boolean frontSensorOut() {
    return (front_Sensor.getRange() < 300);
  }

  public boolean backSensorOut() {
    return (shoot_Sensor.getRange() < 300);
  }

  public boolean invbackSensorOut() {
    return !(shoot_Sensor.getRange() < 300);
  }

  public void autoIntake() {
    System.out.println(front_Sensor.getRange());
    if (!frontSensorOut()) {
      intakeOnShoot();
    } else {
      intakeOff();
    }
  }

  public Command disableIntake() {
    return runOnce(
            () -> {
              intake.disable();
              feedFront.disable();
              feedBack.disable();
            })
        .andThen(run(() -> {}).withTimeout(0.03))
        .withName("Idle");
  }
}
