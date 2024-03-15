// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED.LEDState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.intake.Intake;

public class GroundIntakeToShooter extends Command {

  private Intake intake;
  private Shooter shooter;

  /** Creates a new GroundIntakeToShooter */
  public GroundIntakeToShooter(Intake intake, Shooter shooter) {

    this.intake = intake;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.lowerToIntake();
     LED.getInstance().changeLedState(LEDState.INTAKING_SPEAKER);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (!shooter.shooterSensorOut()) {
      intake.intakeToShooter();
      if(intake.intakeSensorOut()) {
        LED.getInstance().changeLedState(LEDState.NOTE_IN_FEED);
      } else {
        LED.getInstance().changeLedState(LEDState.INTAKING_SPEAKER);
      }
    } else {
      intake.disableIntake();
      LED.getInstance().changeLedState(LEDState.INTAKE_SUCCESS_SPEAKER);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.disableIntake();
    shooter.stowShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shooter.shooterSensorOut()) {
      return true;
    }
    return false;
  }
}
