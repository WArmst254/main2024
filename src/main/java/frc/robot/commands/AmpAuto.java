package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.elevator.Elevator;

public class AmpAuto {

  public static Command ampAuto() {
    // Subsystems
    Amp amp = new Amp();
    Elevator elevator = new Elevator();

    return (Commands.run(() -> elevator.autoAmpElevator()) // raise elevator to amp scoring position
        .andThen(
            () -> {
              if (amp.ampSensorOut()) {
                // if the amp sensor is triggered ampSensorOut will return true, indicating that a
                // note is within the amp mechanism
                amp.AmpOuttakeOn(); // outtake
              } else {
                // if the amp sensor returns false, the note has successfully amp outtaked
                amp.AmpOuttakeOff(); // outtake motor off
              }
            })
        .until(
            amp::invAmpSensorOut) // cancel the command when the amp sensor is no longer triggered
        .andThen(elevator.autoHomeElevator())); // return elevator to home position
  }
}
