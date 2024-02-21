package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private final TalonFX elevator = new TalonFX(20);
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

  public Elevator() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    /* Configure current limits */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 100; // 5 rotations per second cruise
    mm.MotionMagicAcceleration = 100; // Take approximately 0.5 seconds to reach max vel
    mm.MotionMagicJerk = 180; // Take approximately 0.2 seconds to reach max accel

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = 100;
    slot0.kI = .2;
    slot0.kD = 0.4;
    slot0.kV = 1.2;
    slot0.kS = 1; // Approximately 0.25V to get the mechanism moving

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 52;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = elevator.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }

  public void periodic() {
    SmartDashboard.putNumber("Elevator Position: ", elevator.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Velocity: ", elevator.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Power:", elevator.get());
    SmartDashboard.putNumber("Elevator Voltage:", elevator.getMotorVoltage().getValueAsDouble());
  }

  public void setElevatorPosition() {
    elevator.setControl(m_mmReq.withPosition(-1).withSlot(0));
  }

  public void homeElevator() {
    elevator.setControl(m_mmReq.withPosition(0).withSlot(0));
  }

  public void zeroElevatorPosition() {
    elevator.setPosition(0);
  }

  public void setElevatorHomePosition() {
    elevator.set(.2);
    if (elevator.getMotorVoltage().getValueAsDouble() > .5) {
      zeroElevatorPosition();
      elevator.set(0);
    }
  }

  public Command autoAmpElevator() {
    return runOnce(
            () -> {
              elevator.setControl(m_mmReq.withPosition(-0.8).withSlot(0));
            })
        .andThen(run(() -> {}).withTimeout(0.5))
        .withName("Elevator Lifted");
  }

  public Command autoHomeElevator() {
    return runOnce(
            () -> {
              elevator.setControl(m_mmReq.withPosition(0).withSlot(0));
            })
        .andThen(run(() -> {}).withTimeout(0.5))
        .withName("Elevator Lifted");
  }
}
