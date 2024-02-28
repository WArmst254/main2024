package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final TalonFX intake = new TalonFX(Constants.IDs.intake);
  private final TalonFX frontFeed = new TalonFX(Constants.IDs.frontFeed);
  private final TalonFX backFeed = new TalonFX(Constants.IDs.backFeed);
  public final TimeOfFlight intake_sensor = new TimeOfFlight(Constants.IDs.intakesensor);

  public Intake() {
    setDefaultCommand(
        runOnce(
                () -> {
                  intake.set(0);
                  frontFeed.set(0);
                  backFeed.set(0);
                })
            .andThen(run(() -> {}))
            .withName("Intake Idle"));
  }

  public void intakeToShooter() {
    intake.set(1); //TODO: Probably Change
    frontFeed.set(-0.95);
    backFeed.set(0.95);
  }

  public void intakeToAmp() {
    intake.set(1);
    frontFeed.set(-0.95);
    backFeed.set(-0.95);
  }

  public void feedHPIntakeToGroundIntake() {
    frontFeed.set(0.95);
    backFeed.set(-0.95);
  }

  public void outakeFromShooter() {
    intake.set(-.95);
    frontFeed.set(0.95);
    backFeed.set(-0.95);
  }

  public void outakeFromAmp() {
    intake.set(-1);
    frontFeed.set(0.95);
    backFeed.set(0.95);
  }

  public void disableIntake() {
    intake.set(0);
    frontFeed.set(0);
    backFeed.set(0);
  }

  public void disableBackFeed() {
    frontFeed.set(0);
  }

  public void disableFeeds() {
    backFeed.set(0);
    frontFeed.set(0);
  }

  public void intakeOn() {
    intake.set(1);
  }

  public void backFeedOn() {
    frontFeed.set(-0.5);

  }

  public double intakeSensor() {
    return intake_sensor.getRange();
  }

  public boolean intakeSensorOut() {
    return (intake_sensor.getRange() < 300);
  }

  public boolean invIntakeSensorOut() {
    return !(intake_sensor.getRange() < 300);
  }

  public void periodic() {
    SmartDashboard.putNumber("Intake Power: ", intake.get());
    SmartDashboard.putNumber("Front Feed Power: ", frontFeed.get());
    SmartDashboard.putNumber("Back Feed Power:", backFeed.get());
  }
}
