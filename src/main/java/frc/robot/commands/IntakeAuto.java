package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class IntakeAuto {

  public static Command intakeShootAuto() {
    // subsystems
    Intake intake = new Intake();
    Shooter shooter = new Shooter();

    return (Commands.run(
            () -> {
              if (!shooter.shooterSensorOut()) {
                // if the shooter sensor is not triggered shooterSensorOut will return false,
                // indicating that the note has not yet reached the shooter mechanism
                intake.intakeOnShoot(); // intake and feed to shooter mechanism
              } else {
                intake.intakeOff(); // intake and feed motors off
              }
            })
        .until(
            shooter::shooterSensorOut)); // cancel the command when the shooter sensor is triggered
  }

  public static Command intakeAmpAuto() {
    // subsystems
    Intake intake = new Intake();
    Amp amp = new Amp();

    return (Commands.run(
            () -> {
              if (!amp.ampSensorOut()) {
                // if the amp sensor is not triggered, ampSensorOut will return false, indicating
                // that the note has not yet reached the amp mechanism
                intake.intakeOnAmp(); // intake and feed to amp mechanism
                amp.AmpOuttakeOn(); // run amp motors
              } else {
                intake.intakeOff(); // intake and feed motors off
                amp.AmpOuttakeOff(); // amp motors off
              }
            })
        .until(amp::ampSensorOut)); // cancel the command when the amp sensor is triggered
  }

  public static Command outakeShootAuto() {
    // subsytems
    Intake intake = new Intake();

    return (Commands.run(
            () -> {
              if (intake.intakeSensorOut()) {
                // if the intake sensor is triggered intakeSensorOut will return true, indicating
                // that a note is within the intake mechanism
                intake.outakeOnShoot(); // outtake through shooter/back feeds
              } else {
                // if the intake sensor returns false, the note has successfully outtaked
                intake.intakeOff(); // outtake motors off
              }
            })
        .until(
            intake::invIntakeSensorOut)); // cancel the command when the intake sensor is no longer
    // triggered
  }

  public static Command outakeAmpAuto() {
    // subsystems
    Intake intake = new Intake();

    return (Commands.run(
            () -> {
              if (intake.intakeSensorOut()) {
                // if the intake sensor is triggered intakeSensorOut will return true, indicating
                // that a note is within the intake mechanism
                intake.outakeOnAmp(); // outtake through amp feeds
              } else {
                // if the intake sensor returns false, the note has successfully outtaked
                intake.intakeOff(); // outtake motors off
              }
            })
        .until(
            intake::invIntakeSensorOut)); // cancel the command when the intake sensor is no longer
    // triggered
  }
}
