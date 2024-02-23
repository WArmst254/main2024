package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;

public class ShooterCommands {
  public static Command shootSensorCommand(Shooter shooter, Intake intake) {
    // the parallel command type will run both of these command threads simultaneously
    return Commands.parallel(
            Commands.run(
                    () -> // first command thread
                    shooter.shooterOn(70)) // activates shooter velocity closed-loop
                .until(shooter::invShooterSensorOut), // cancel command when the note has been shot
            Commands.waitUntil(
                    shooter::isShooterSet) // second command thread that begins once target velocity is achieved
                .andThen(() -> intake.backFeedOn()) // activates feed to shoot note
                .until(shooter::invShooterSensorOut)) // cancel command when the note has been shot
        .withName("Shoot");
  }

  public static Command fullInterpolatedShootCommand(Shooter shooter, Intake intake, Vision vision) {
    // the parallel command type will run both of these command threads simultaneously
    return Commands.parallel(
            Commands.runEnd(
                () -> {shooter.interpolatedShootAngleCommand(vision);},
                 // first command thread
                () -> {shooter.interpolatedShooterOnCommand(vision);})
                 // activates shooter velocity closed-loop
                .until(shooter::invShooterSensorOut), // cancel command when the note has been shot
            Commands.waitUntil(shooter::isShooterSet) // second command thread that begins once target velocity is achieved
                .andThen(() -> intake.backFeedOn()) // activates feed to shoot note
                .until(shooter::invShooterSensorOut)) // cancel command when the note has been shot
        .withName("Shoot");
  }

  public static Command shootManualCommand(Shooter shooter, Intake intake) {
    return Commands.parallel(
            Commands.run(
                () -> // first command thread
                shooter.shooterOn(70)), // activates shooter velocity closed-loop
            Commands.waitUntil(
                    shooter::isShooterSet) // second command thread that begins once target velocity is achieved
                .andThen(() -> intake.backFeedOn())) // activates feed to shoot note
        .withName("Shoot");
  }
}
