package frc.robot.subsystems.shooter;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final CANSparkFlex shooterLeft =
      new CANSparkFlex(19, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkFlex shooterRight =
      new CANSparkFlex(20, CANSparkLowLevel.MotorType.kBrushless);
  private final TalonFX feedBack = new TalonFX(13);
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

    // Set default command to turn off both the shooter and feeder motors, and then idle
    setDefaultCommand(
        runOnce(
                () -> {
                  shooterLeft.disable();
                  feedBack.disable();
                })
            .andThen(run(() -> {}))
            .withName("Idle"));
  }

  public Command teleshootCommand(double setpointRotationsPerSecond) {
    return parallel(
            // Run the shooter at the desired setpoint using feedforward and feedback
            run(
                () ->
                    shooterLeft.set(
                        m_shooterFeedforward.calculate(setpointRotationsPerSecond)
                            + m_leftShooterFeedback.calculate(
                                m_leftencoder.getVelocity() / 60, setpointRotationsPerSecond))),
            // Wait until shooter motor have reached the setpoint, and then run the feeder
            waitUntil(m_leftShooterFeedback::atSetpoint).andThen(() -> feedBack.set(-0.45)))
        .withName("Shoot");
  }

  public Command shootCommand(double setpointRotationsPerSecond) {
    return parallel(
            // Run the shooter at the desired setpoint using feedforward and feedback
            run(() ->
                    shooterLeft.set(
                        m_shooterFeedforward.calculate(setpointRotationsPerSecond)
                            + m_leftShooterFeedback.calculate(
                                m_leftencoder.getVelocity() / 60, setpointRotationsPerSecond)))
                .withTimeout(1),
            // Wait until shooter motor have reached the setpoint, and then run the feeder
            waitUntil(m_leftShooterFeedback::atSetpoint)
                .andThen(() -> feedBack.set(-0.45))
                .withTimeout(1))
        .withName("Shoot");
  }

  // public Command shootCommandSensor(double setpointRotationsPerSecond) {
  //   return parallel(
  //           // Run the shooter at the desired setpoint using feedforward and feedback
  //           run(() ->
  //                   shooterLeft.set(
  //                       m_shooterFeedforward.calculate(setpointRotationsPerSecond)
  //                           + m_leftShooterFeedback.calculate(
  //                               m_leftencoder.getVelocity() / 60, setpointRotationsPerSecond)))
  //               .until(intake::invbackSensorOut),
  //           // Wait until shooter motor have reached the setpoint, and then run the feeder
  //           waitUntil(m_leftShooterFeedback::atSetpoint)
  //               .andThen(() -> feedBack.set(-0.45))
  //               .until(intake::invbackSensorOut))
  //       .withName("Shoot");
  // }

  public Command disableShooter() {
    return runOnce(
            () -> {
              shooterLeft.disable();
              feedBack.disable();
            })
        .andThen(run(() -> {}).withTimeout(0.05))
        .withName("Idle");
  }
}
