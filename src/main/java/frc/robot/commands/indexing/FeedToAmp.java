// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED.LEDState;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.intake.Intake;

public class FeedToAmp extends Command {

  private Intake intake;
  private Amp amp;

  /** Creates a new FeedToAmp */
  public FeedToAmp(Intake intake, Amp amp) {

    this.intake = intake;
    this.amp = amp;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, amp);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LED.getInstance().changeLedState(LEDState.INTAKING_AMP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!amp.ampSensorOut()) {
      intake.intakeToAmp();
      amp.ampOuttakeOn();
    } else {
      intake.disableIntake();
      amp.disableAmp();
      LED.getInstance().changeLedState(LEDState.INTAKE_SUCCESS_AMP);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.disableIntake();
    amp.disableAmp();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (amp.ampSensorOut()) {
      return true;
    }
    return false;
  }
}
