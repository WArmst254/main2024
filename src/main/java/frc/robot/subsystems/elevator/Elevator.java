package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private final TalonFX elevator = new TalonFX(20);
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

  public Elevator() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    /* Configure current limits */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 5; // 5 rotations per second cruise
    mm.MotionMagicAcceleration = 10; // Take approximately 0.5 seconds to reach max vel
    // Take approximately 0.2 seconds to reach max accel
    mm.MotionMagicJerk = 50;

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = 10;
    slot0.kI = 0;
    slot0.kD = 0.1;
    slot0.kV = 0.12;
    slot0.kS = 0.25; // Approximately 0.25V to get the mechanism moving

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 12.8;

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
    SmartDashboard.putNumber("Elevator Pos: ", elevator.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Vel: ", elevator.getVelocity().getValueAsDouble());
    System.out.println();
  }

  public void setElevatorPosition() {
    elevator.setControl(m_mmReq.withPosition(-1.2).withSlot(0));
  }

  public void homeElevator() {
    elevator.setControl(m_mmReq.withPosition(0).withSlot(0));
  }

  public void zeroElevatorPosition() {
    elevator.setPosition(0);
  }
}
