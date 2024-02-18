package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
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
    return (front_Sensor.getRange() < 375);
  }

  public boolean backSensorOut() {
    return (shoot_Sensor.getRange() < 350);
  }

  public void autoIntake() {
    System.out.println(front_Sensor.getRange());
    if (!frontSensorOut()) {
      intakeOnShoot();
    } else {
      intakeOff();
    }
  }
}
