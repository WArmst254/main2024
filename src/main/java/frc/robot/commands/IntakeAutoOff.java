package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.intake.Intake;

public class IntakeAutoOff {
  // Intake intake = new Intake();
  public static Command intakeShootAuto() {
    Intake intake = new Intake();

    return (Commands.run(
            () -> {
              if (!intake.backSensorOut()) {
                intake.intakeOnShoot();
              } else {
                intake.intakeOff();
              }
            })
        .until(intake::backSensorOut));
  }

  public static Command intakeAmpAuto() {
    Intake intake = new Intake();
    Amp amp = new Amp();
    return (Commands.run(
            () -> {
              if (!amp.ampSensorOut()) {
                intake.intakeOnAmp();
                amp.AmpOuttakeOn();
              } else {
                intake.intakeOff();
                amp.AmpOuttakeOff();
              }
            })
        .until(amp::ampSensorOut));
  }
}
