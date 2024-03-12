// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.indexing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED.LEDState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.intake.Intake;

public class FeedToShooter extends Command {

  private Intake intake;
  private Shooter shooter;

  /** Creates a new FeedToShooter */
  public FeedToShooter(Intake intake, Shooter shooter) {

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
      intake.feedToShooter();
    } else {
      intake.disableFeeds();
      LED.getInstance().changeLedState(LEDState.INTAKE_SUCCESS_SPEAKER);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.disableFeeds();
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
