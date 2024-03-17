package frc.robot.subsystems.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TunableNumber;

public class Intake extends SubsystemBase {
  private TunableNumber intakeEntrySpeed = new TunableNumber("Intake/Intake Entry Speed");
   private TunableNumber intakeFeedSpeed = new TunableNumber("Intake/Intake Feed Speed");
   private TunableNumber outakeExitSpeed = new TunableNumber("Intake/Outake Exit Speed");
   private TunableNumber outakeFeedSpeed = new TunableNumber("Intake/Outake Feed Speed");
   private TunableNumber pushToShootFeedSpeed = new TunableNumber("Shooter/Feed Speed");
   private TunableNumber intakeSensorThreshold = new TunableNumber("Sensors/Intake Sensor Threshold");

  private final TalonFX intake;
  private final TalonFX frontFeed;
  private final TalonFX backFeed;
  private final TimeOfFlight intake_sensor;

  public Intake() {
    intake = new TalonFX(Constants.IDs.intake);
    frontFeed = new TalonFX(Constants.IDs.frontFeed);
    backFeed = new TalonFX(Constants.IDs.backFeed);

    intakeEntrySpeed.setDefault(IntakeConstants.entrySpeed);
    intakeFeedSpeed.setDefault(IntakeConstants.intakeFeedSpeed);
    outakeExitSpeed.setDefault(IntakeConstants.exitSpeed);
    outakeFeedSpeed.setDefault(IntakeConstants.outakeFeedSpeed);
    pushToShootFeedSpeed.setDefault(IntakeConstants.pushSpeed);

    intake_sensor = new TimeOfFlight(Constants.IDs.intakesensor);
    intake_sensor.setRangingMode(TimeOfFlight.RangingMode.Short, 0.001);
    intake_sensor.setRangeOfInterest(6, 6, 14, 14);
    intakeSensorThreshold.setDefault(IntakeConstants.sensorThreshold);

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
    intake.set(intakeEntrySpeed.get());
    frontFeed.set(-(intakeFeedSpeed.get()));
    backFeed.set(intakeFeedSpeed.get());
  }

  public void feedToShooter() {
    frontFeed.set(-(intakeFeedSpeed.get()));
    backFeed.set(intakeFeedSpeed.get());
  }

  public void feedSlowToShooter() {
    frontFeed.set(-0.1);
    backFeed.set(0.1);
  }

  public void intakeToAmp() {
    intake.set(intakeEntrySpeed.get());
    frontFeed.set(-(intakeFeedSpeed.get()));
    backFeed.set(-(intakeFeedSpeed.get()));
  }


  public void feedFromShooter() {
    frontFeed.set(outakeFeedSpeed.get());
    backFeed.set(-(outakeFeedSpeed.get()));
  }

  public void feedFromAmp(){
    frontFeed.set(outakeFeedSpeed.get());
    backFeed.set(-(outakeFeedSpeed.get()));
}

  public void outakeFromShooter() {
    intake.set(-(outakeExitSpeed.get()));
    frontFeed.set(outakeFeedSpeed.get());
    backFeed.set(-(outakeFeedSpeed.get()));
  }

  public void outakeFromAmp() {
    intake.set(-(outakeExitSpeed.get()));
    frontFeed.set(-(outakeFeedSpeed.get()));
    backFeed.set(-(outakeFeedSpeed.get()));
  }

  public void disableIntake() {
    intake.set(0);
    frontFeed.set(0);
    backFeed.set(0);
  }

  public void disableFeeds() {
    backFeed.set(0);
    frontFeed.set(0);
  }

  public void intakeOn() {
    intake.set(intakeEntrySpeed.get());
  }

  public void pushToShoot() {
    frontFeed.set(-(pushToShootFeedSpeed.get()));
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
