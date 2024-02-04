package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {

  private final TalonFX elevator = new TalonFX(15);

  public Elevator() {
    var config = new MotorOutputConfigs();
    config.NeutralMode = NeutralModeValue.Brake;
    elevator.getConfigurator().apply(config);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kP = 24; // An error of 0.5 rotations results in 12 V output
    slot0Configs.kI = 0; // no output for integrated error
    slot0Configs.kD = 0.1; // A velocity of 1 rps results in 0.1 V output

    elevator.getConfigurator().apply(slot0Configs);

    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);

    // set position to 10 rotations
    elevator.setControl(m_request.withPosition(10));
  }
}
