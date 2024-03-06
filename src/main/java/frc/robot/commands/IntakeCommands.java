package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class IntakeCommands {

  public static Command intakeToShooterSensorCommand(
      Intake intake, Shooter shooter, double lowerAngle) {
    return (Commands.run(
            () -> {
              if (!shooter.shooterSensorOut()) {
                intake.intakeToShooter(); // intake and feed to shooter mechanism
                shooter.lowerShooter(lowerAngle);
              } else {
                intake.disableIntake(); // intake and feed motors off
                shooter.stowShooter();
              }
            })
        .until(
            shooter::shooterSensorOut)); // cancel the command when the shooter sensor is triggered
  }

  public static Command intakeToAmpSensorCommand(Intake intake, Amp amp) {

    return (Commands.run(
            () -> {
              if (!amp.ampSensorOut()) {
                // if the amp sensor is not triggered, ampSensorOut will return false, indicating that the note has not yet reached the amp mechanism
                intake.intakeToAmp(); // intake and feed to amp mechanism
                amp.ampOuttakeOn(); // run amp motors
              } else {
                intake.disableIntake(); // intake and feed motors off
                amp.disableAmp(); // amp motors off
              }
            })
        .until(amp::ampSensorOut)); // cancel the command when the amp sensor is triggered
  }

  public static Command outakeFromShooterSensorCommand(Intake intake, Shooter shooter) {

    return (Commands.run(
            () -> {
              if(!intake.intakeSensorOut()) {
                shooter.intakeHP();
                intake.outakeFromShooter();
              } else {
                shooter.disableFlywheels();
                intake.outakeFromShooter();
              }
            }));
  }

  public static Command outakeFromAmpSensorCommand(Intake intake, Amp amp) {

    return (Commands.run(
            () -> {
              if (!intake.intakeSensorOut()) {
                amp.ampIntakeOn();
                intake.outakeFromAmp();
              } else {
                amp.disableAmp();
                intake.outakeFromAmp();
              }
            }));
  }
}
