package frc.robot.subsystems.shootangle;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ShootAngle extends SubsystemBase {

  private final TalonFX shootAngle = new TalonFX(16);
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

  public ShootAngle() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();
    shootAngle.setNeutralMode(NeutralModeValue.Brake);

    /* Configure current limits */
    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = 10;
    mm.MotionMagicAcceleration = 10;
    mm.MotionMagicJerk = 50;

    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = 11;
    slot0.kI = 0.1;
    slot0.kD = 0.1;
    slot0.kV = 0.12;
    slot0.kS = 0.25;

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 12.8;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = shootAngle.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }

  public void periodic() {
    SmartDashboard.putNumber("Shoot Angle Position: ", shootAngle.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Shoot Angle Velocity: ", shootAngle.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shoot Angle Power:", shootAngle.get());
    SmartDashboard.putNumber(
        "Shoot Angle Voltage:", shootAngle.getMotorVoltage().getValueAsDouble());
  }

  public void lowerShootAngle() {
    shootAngle.setControl(m_mmReq.withPosition(0.18).withSlot(0));
  }

  public void stowShootAngle() {
    shootAngle.setControl(m_mmReq.withPosition(0).withSlot(0));
  }

  public void zeroShootAngle() {
    shootAngle.setPosition(0);
  }
}
