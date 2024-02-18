package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.intake.Intake;

public class IntakeAutoOff {
  // Intake intake = new Intake();
  public static Command intakeAuto() {
    Intake intake = new Intake();
    return (Commands.run(
        () -> {
          if (!intake.frontSensorOut()) {
            intake.intakeOnShoot();
          } else {
            intake.intakeOff();
          }
        }));
  }
}
