package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.elevator.Elevator;

public class AmpCommands {

  public static Command ampAutonomousCommand(Amp amp, Elevator elevator) {

    return (Commands.run(
            () -> elevator.ampElevatorCommand()) // raise elevator to amp scoring position
        .andThen(
            () -> {
              if (amp.ampSensorOut()) {
                // if the amp sensor is triggered ampSensorOut will return true, indicating that anote is within the amp mechanism
                amp.ampOuttakeOn(); // outtake
              } else {
                // if the amp sensor returns false, the note has successfully amp outtaked
                amp.disableAmp(); // outtake motor off
              }
            })
        .until(
            amp::invAmpSensorOut) // cancel the command when the amp sensor is no longer triggered
        .andThen(elevator.stowElevatorCommand())); // return elevator to home position
  }

  public static Command ampTeleopCommand(Amp amp) {

    return (Commands.run(
            () -> {
              if (amp.ampSensorOut()) {
                // if the amp sensor is triggered ampSensorOut will return true, indicating that a note is within the amp mechanism
                amp.ampOuttakeOn(); // outtake
              } else {
                // if the amp sensor returns false, the note has successfully amp outtaked
                amp.disableAmp(); // outtake motor off
              }
            })
        .until(
            amp::invAmpSensorOut)); // cancel the command when the amp sensor is no longer triggered
  }
}
