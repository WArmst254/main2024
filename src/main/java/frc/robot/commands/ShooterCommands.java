package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.Shooter.State;

public class ShooterCommands {
  public static Command shootSensorCommand(Shooter shooter, Intake intake, double angle, double setpointRotationsPerSecond) {
    // the parallel command type will run both of these command threads simultaneously
    return Commands.parallel(
        Commands.parallel(
            Commands.run(() -> shooter.lowerShootAngle(angle)),
            Commands.run(() -> shooter.shooterOn(setpointRotationsPerSecond)))
                .until(shooter::invShooterSensorOut), // cancel command when the note has been shot
            Commands.waitUntil(
                    shooter::isShooterSet) // second command thread that begins once target velocity is achieved
                .andThen(() -> intake.backFeedOn()) // activates feed to shoot note
                .until(shooter::invShooterSensorOut)) // cancel command when the note has been shot
        .withName("Shoot");
  }

  
  public static Command interpolatedShootSensorCommand(Shooter shooter, Intake intake, State state) {
        // the parallel command type will run both of these command threads simultaneously
        return Commands.parallel(
                Commands.parallel(
                    Commands.run(() -> shooter.interpolatedShootAngle(state)),
                     // first command thread
                    Commands.run(() -> shooter.interpolatedShooterVelocity(state))),
                     // activates shooter velocity closed-loop
                Commands.waitUntil(shooter::isShooterSet) // second command thread that begins once target velocity is achieved
                    .andThen(() -> intake.backFeedOn()) // activates feed to shoot note
                    .until(shooter::invShooterSensorOut)) // cancel command when the note has been shot
            .withName("Shoot");
      }

      public static Command interpolatedShootManualCommand(Shooter shooter, Intake intake, State state) {
        // the parallel command type will run both of these command threads simultaneously
        return Commands.parallel(
                Commands.parallel(
                    Commands.run(() -> shooter.interpolatedShootAngle(state)),
                     // first command thread
                    Commands.run(() -> shooter.interpolatedShooterVelocity(state))),
                     // activates shooter velocity closed-loop
                Commands.waitUntil(shooter::isShooterSet) // second command thread that begins once target velocity is achieved
                    .andThen(() -> intake.backFeedOn())) // cancel command when the note has been shot
            .withName("Shoot");
      }



  public static Command shootManualCommand(Shooter shooter, Intake intake, double angle, double setpointRotationsPerSecond) {
    return Commands.parallel(
                Commands.parallel(
            Commands.run(() -> shooter.lowerShootAngle(angle)),
            Commands.run(() -> shooter.shooterOn(setpointRotationsPerSecond))), // activates shooter velocity closed-loop
            Commands.waitUntil(
                    shooter::isShooterSet) // second command thread that begins once target velocity is achieved
                .andThen(() -> intake.backFeedOn())) // activates feed to shoot note
        .withName("Shoot");
  }

  public static Command HPintakeToAmpSensorCommand(Intake intake, Amp amp, Shooter shooter) {

    return (Commands.run(
            () -> {
              if (!amp.ampSensorOut()) {
                // if the amp sensor is not triggered, ampSensorOut will return false, indicating that the note has not yet reached the amp mechanism
                shooter.intakeHP();
                intake.feedHPIntakeToAmp(); // intake and feed to amp mechanism
                amp.ampOuttakeOn(); // run amp motors
              } else {
                shooter.disableShooter();
                intake.disableFeeds(); // intake and feed motors off
                amp.disableAmp(); // amp motors off
              }
            })
        .until(amp::invAmpSensorOut)); // cancel the command when the amp sensor is triggered
  }



}
