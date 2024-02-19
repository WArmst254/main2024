package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.amp.Amp;

public class AmpAutoOff {
  // Intake intake = new Intake();
  public static Command ampAuto() {
    Amp amp = new Amp();
    return (Commands.run(
            () -> {
              if (!amp.invampSensorOut()) {
                amp.AmpOuttakeOn();
              } else {
                amp.AmpOuttakeOff();
              }
            })
        .until(amp::invampSensorOut));
  }
}
