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
import frc.robot.Constants;

public class Elevator extends SubsystemBase {

  private final TalonFX elevator = new TalonFX(Constants.IDs.elevator);
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

  public Elevator() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 100;
    mm.MotionMagicAcceleration = 100;
    mm.MotionMagicJerk = 180;

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = 100;
    slot0.kI = .2;
    slot0.kD = 0.4;
    slot0.kV = 1.2;
    slot0.kS = 1;

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

  public void fullExtendElevator() {
    elevator.setControl(m_mmReq.withPosition(-1).withSlot(0));
  }

  public void ampExtendElevator() {
    elevator.setControl(m_mmReq.withPosition(-0.8).withSlot(0));
  }

  public void stowElevator() {
    elevator.setControl(m_mmReq.withPosition(0).withSlot(0));
  }

  public void zeroElevatorPosition() {
    elevator.setPosition(0);
  }

  public void setElevatorStowPosition() {
    elevator.set(.2);
    if (elevator.getMotorVoltage().getValueAsDouble() > .5) {
      zeroElevatorPosition();
      elevator.set(0);
    }
  }

  public boolean isElevatorSet() {
    return (m_mmReq.Position <= (elevator.getPosition().getValueAsDouble()+0.01) && m_mmReq.Position >= (elevator.getPosition().getValueAsDouble()-0.01));
  }

  public Command ampElevatorCommand() {
    return runOnce(
            () -> {
              elevator.setControl(m_mmReq.withPosition(-0.8).withSlot(0));
            })
        .andThen(run(() -> {}).withTimeout(0.5))
        .withName("Elevator Lifted");
  }

  public Command stowElevatorCommand() {
    return runOnce(
            () -> {
              elevator.setControl(m_mmReq.withPosition(0).withSlot(0));
            })
        .andThen(run(() -> {}).withTimeout(0.5))
        .withName("Elevator Stowed");
  }
}
