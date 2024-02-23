package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.shooter.Shooter;

public class ShootAuto {
  public static Command shootAuto(Shooter shooter) {
    // the parallel command type will run both of these command threads simultaneously
    return Commands.parallel(
            Commands.run(
                    () -> // first command thread
                    shooter.shooterOn(70)) // activates shooter velocity closed-loop
                .until(shooter::invShooterSensorOut), // cancel command when the note has been shot
            Commands.waitUntil(
                    shooter
                        ::isShooterSet) // second command thread that begins once target velocity is
                // achieved
                .andThen(() -> shooter.backFeedOn()) // activates feed to shoot note
                .until(shooter::invShooterSensorOut)) // cancel command when the note has been shot
        .withName("Shoot");
  }
}
