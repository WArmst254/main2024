// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED.LEDState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.intake.Intake;

public class IntakeNoteSpeaker extends Command {

  private Intake intake;
  private Shooter shooter;

  /** Creates a new IntakeNote. */
  public IntakeNoteSpeaker(Intake intake, Shooter shooter) {

    this.intake = intake;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.lowerShooter(0.15);
     LED.getInstance().changeLedState(LEDState.INTAKING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (shooter.invShooterSensorOut()) {
      intake.intakeToShooter();
    } else {
      intake.disableIntake();
      LED.getInstance().changeLedState(LEDState.GROUND_INTAKE_DETECTED);
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
