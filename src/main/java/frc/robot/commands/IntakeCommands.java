package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shootangle.ShootAngle;
import frc.robot.subsystems.shooter.Shooter;

public class IntakeCommands {

  public static Command intakeToShooterSensorCommand(Intake intake, Shooter shooter, ShootAngle shootAngle) {
    return (Commands.run(
            () -> {
              if (!shooter.shooterSensorOut()) {
                // if the shooter sensor is not triggered shooterSensorOut will return false,
                // indicating that the note has not yet reached the shooter mechanism

                intake.intakeToShooter(); // intake and feed to shooter mechanism
                shootAngle.lowerShootAngle();
              } else {
                intake.intakeToShooter(); // intake and feed motors off
                shootAngle.lowerShootAngle();
              }
            })
        .until(
            shooter::shooterSensorOut)); // cancel the command when the shooter sensor is triggered
  }

  public static Command intakeToAmpSensorCommand(Intake intake, Amp amp) {

    return (Commands.run(
            () -> {
              if (!amp.ampSensorOut()) {
                // if the amp sensor is not triggered, ampSensorOut will return false, indicating
                // that the note has not yet reached the amp mechanism
                intake.intakeToAmp(); // intake and feed to amp mechanism
                amp.ampOuttakeOn(); // run amp motors
              } else {
                intake.intakeToAmp(); // intake and feed motors off
                amp.disableAmp(); // amp motors off
              }
            })
        .until(amp::invAmpSensorOut)); // cancel the command when the amp sensor is triggered
  }

  public static Command outakeFromShooterSensorCommand(Intake intake) {

    return (Commands.run(
            () -> {
              if (intake.intakeSensorOut()) {
                // if the intake sensor is triggered intakeSensorOut will return true, indicating
                // that a note is within the intake mechanism
                intake.outakeFromShooter(); // outtake through shooter/back feeds
              } else {
                // if the intake sensor returns false, the note has successfully outtaked
                intake.disableIntake(); // outtake motors off
              }
            })
        .until(intake::invIntakeSensorOut)); // cancel the command when the intake sensor is no longer
    // triggered
  }

  public static Command outakeFromAmpSensorCommand(Intake intake) {

    return (Commands.run(
            () -> {
              if (!intake.intakeSensorOut()) {
                // if the intake sensor is triggered intakeSensorOut will return true, indicating
                // that a note is within the intake mechanism
                intake.outakeFromAmp(); // outtake through amp feeds
              } else {
                // if the intake sensor returns false, the note has successfully outtaked
                intake.disableIntake(); // outtake motors off
              }
            })
        .until(intake::intakeSensorOut)); // cancel the command when the intake sensor is no longer
    // triggered
  }
}