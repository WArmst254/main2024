package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final CANSparkFlex shooterLeft = new CANSparkFlex(19, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkFlex shooterRight = new CANSparkFlex(20, CANSparkLowLevel.MotorType.kBrushless);
  private final TalonFX backFeed = new TalonFX(13);
  public final TimeOfFlight shooter_sensor = new TimeOfFlight(2);
  private RelativeEncoder m_leftencoder = shooterLeft.getEncoder();

  private final SimpleMotorFeedforward m_shooterFeedforward =
      new SimpleMotorFeedforward(0.05, 12 / 6000);

  private final PIDController m_leftShooterFeedback = new PIDController(0.09, 0.0, 0.0);

  public Shooter() {
    shooterLeft.restoreFactoryDefaults();
    shooterRight.restoreFactoryDefaults();
    shooterRight.follow(shooterLeft);
    shooterRight.burnFlash();
    m_leftShooterFeedback.setTolerance(25);

    setDefaultCommand(
        runOnce(
                () -> {
                  shooterLeft.disable();
                  backFeed.disable();
                })
            .andThen(run(() -> {}))
            .withName("Shooter Idle"));
  }

  public void shooterOn(double setpointRotationsPerSecond) {
    shooterLeft.set(m_shooterFeedforward.calculate(setpointRotationsPerSecond)
                      + m_leftShooterFeedback.calculate(
                          m_leftencoder.getVelocity() / 60, setpointRotationsPerSecond));
  }

  public void backFeedOn(){
    backFeed.set(-0.45);
  }

  public boolean shooterSensorOut() {
    return (shooter_sensor.getRange() < 300);
  }

  public boolean invShooterSensorOut() {
    return !(shooter_sensor.getRange() < 300);
  }

  public boolean isShooterSet() {
    return (m_leftShooterFeedback.atSetpoint());
  }

  public Command disableShooter() {
    return runOnce(
            () -> {
              shooterLeft.disable();
              backFeed.disable();
            })
        .andThen(run(() -> {}).withTimeout(0.01))
        .withName("Shooter Idle");
  }
}
