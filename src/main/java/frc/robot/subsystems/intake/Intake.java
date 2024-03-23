package frc.robot.subsystems.intake;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TunableNumber;

public class Intake extends SubsystemBase {

  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  private final NeutralOut neutralOut = new NeutralOut();

   private TunableNumber intakeSensorThreshold = new TunableNumber("Sensors/Intake Sensor Threshold");

  private final TalonFX intake;
  private final TalonFX frontFeed;
  private final TalonFX backFeed;
  private final TimeOfFlight intake_sensor;

  public Intake() {
    intake = new TalonFX(Constants.IDs.intake);
    frontFeed = new TalonFX(Constants.IDs.frontFeed);
    backFeed = new TalonFX(Constants.IDs.backFeed);

    intake_sensor = new TimeOfFlight(Constants.IDs.intakesensor);
    intake_sensor.setRangingMode(TimeOfFlight.RangingMode.Short, 0.001);
    intake_sensor.setRangeOfInterest(6, 6, 14, 14);
    intakeSensorThreshold.setDefault(IntakeConstants.sensorThreshold);
  }

  public void intakeToShooter() {
    intake.setControl(voltageOut.withOutput(10));
    frontFeed.setControl(voltageOut.withOutput(-10));
    backFeed.setControl(voltageOut.withOutput(4));
  }

  public void feedToShooter() {
    frontFeed.setControl(voltageOut.withOutput(-10));
    backFeed.setControl(voltageOut.withOutput(4));
  }

  public void feedSlowToShooter() {
    frontFeed.setControl(voltageOut.withOutput(-2));
    backFeed.setControl(voltageOut.withOutput(2));
  }

  public void intakeToAmp() {
    intake.setControl(voltageOut.withOutput(10));
    frontFeed.setControl(voltageOut.withOutput(-10));
    backFeed.setControl(voltageOut.withOutput(-10));
  }

  public void feedFromShooter() {
    frontFeed.setControl(voltageOut.withOutput(10));
    backFeed.setControl(voltageOut.withOutput(-10));
  }

  public void feedFromAmp() {
    frontFeed.setControl(voltageOut.withOutput(10));
    backFeed.setControl(voltageOut.withOutput(-10));
}

  public void outakeFromShooter() {
    intake.setControl(voltageOut.withOutput(-10));
    frontFeed.setControl(voltageOut.withOutput(4));
    backFeed.setControl(voltageOut.withOutput(-10));
  }

  public void outakeFromAmp() {
    intake.setControl(voltageOut.withOutput(-10));
    frontFeed.setControl(voltageOut.withOutput(-10));
    backFeed.setControl(voltageOut.withOutput(-10));
  }

  public void disableIntake() {
    intake.setControl(neutralOut);
    frontFeed.setControl(neutralOut);
    backFeed.setControl(neutralOut);
  }

  public void disableFrontRoller() {
    intake.setControl(neutralOut);
  }

  public void disableFeeds() {
    backFeed.setControl(neutralOut);
    frontFeed.setControl(neutralOut);
  }

  public void intakeOn() {
    intake.setControl(voltageOut.withOutput(10));
  }

  public void pushToShoot() {
    frontFeed.setControl(voltageOut.withOutput(-5));
  }

  public double intakeSensor() {
    return intake_sensor.getRange();
  }

  public boolean intakeSensorOut() {
    return (intake_sensor.getRange() < intakeSensorThreshold.get());
  }

  public void periodic() {
    SmartDashboard.putNumber("Intake/Reported Intake Power: ", intake.get());
    SmartDashboard.putNumber("Intake/Reported Front Feed Power: ", frontFeed.get());
    SmartDashboard.putNumber("Intake/Reported Back Feed Power:", backFeed.get());
  }
}
