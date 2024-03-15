package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
//import frc.robot.subsystems.shooter.Shooter.State;
import frc.robot.subsystems.vision.Vision;

public class ShooterCommands {

  public static Command shootSensorCommand(Shooter shooter, Intake intake) {
    return Commands.parallel(
            Commands.run(() -> shooter.flywheelsOnSub()),
            Commands.waitUntil(shooter::isSubwooferVelocitySet)
                .andThen(() -> intake.feedToShooter())
                ).until(shooter::invShooterSensorOut)
        .withName("Shoot");
  }

  public static Command shootSensorCommand(Shooter shooter, Intake intake, Vision vision) {
    return Commands.parallel(
            Commands.run(() -> shooter.interpolatedFlywheelVelocity((shooter.getAutomaticState(vision)))),
            Commands.run(() -> shooter.interpolatedShooterAngle((shooter.getAutomaticState(vision)))),
            Commands.waitUntil(() -> shooter.isInterpolatedVelocitySet(shooter.getAutomaticState(vision)) && shooter.isAngleSet())
                .andThen(() -> intake.feedToShooter())
                ).until(shooter::invShooterSensorOut)
        .withName("Shoot");
  }


  public static Command shootManualCommand(Shooter shooter, Intake intake, double angle, double setpointRotationsPerMinute) {
    return Commands.parallel(
            Commands.run(() -> shooter.flywheelsOnSub()),
            Commands.waitUntil(
                    shooter::isSubwooferVelocitySet)
                .andThen(() -> intake.feedToShooter()))
        .withName("Shoot");
  }

  
  // public static Command interpolatedShootSensorCommand(Shooter shooter, Intake intake, State state) {
  //       // the parallel command type will run both of these command threads simultaneously
  //       return Commands.parallel(
  //               Commands.parallel(
  //                   Commands.run(() -> shooter.interpolatedShootAngle(state)),
  //                    // first command thread
  //                   Commands.run(() -> shooter.interpolatedShooterVelocity(state))),
  //                    // activates shooter velocity closed-loop
  //               Commands.waitUntil(shooter::isShooterSet) // second command thread that begins once target velocity is achieved
  //                   .andThen(() -> intake.backFeedOn()) // activates feed to shoot note
  //                   .until(shooter::invShooterSensorOut)) // cancel command when the note has been shot
  //           .withName("Shoot");
  //     }

  //     public static Command interpolatedShootManualCommand(Shooter shooter, Intake intake, State state) {
  //       // the parallel command type will run both of these command threads simultaneously
  //       return Commands.parallel(
  //               Commands.parallel(
  //                   Commands.run(() -> shooter.interpolatedShootAngle(state)),
  //                    // first command thread
  //                   Commands.run(() -> shooter.interpolatedShooterVelocity(state))),
  //                    // activates shooter velocity closed-loop
  //               Commands.waitUntil(shooter::isShooterSet) // second command thread that begins once target velocity is achieved
  //                   .andThen(() -> intake.backFeedOn())) // cancel command when the note has been shot
  //           .withName("Shoot");
  //     }


  public static Command HPintakeToAmpSensorCommand(Intake intake, Amp amp, Shooter shooter) {

    return (Commands.sequence(
            Commands.run(() -> {
              if (!intake.intakeSensorOut()) {
                shooter.intakeHP();
                intake.feedFromShooter();
              }}
              ).until(intake::intakeSensorOut),
              Commands.run(() -> {
                if (!amp.ampSensorOut()) {
                  shooter.intakeHP();
                  intake.feedFromShooter();
                 }} ).until(amp::ampSensorOut)));
  }
}