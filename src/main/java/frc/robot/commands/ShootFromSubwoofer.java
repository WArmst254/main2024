// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED.LEDState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.intake.Intake;

public class ShootFromSubwoofer extends Command {

  private Intake intake;
  private Shooter shooter;

  /** Creates a new IntakeNote. */
  public ShootFromSubwoofer(Intake intake, Shooter shooter) {

    this.intake = intake;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.lowerToShoot();
     LED.getInstance().changeLedState(LEDState.SUBWOOFER_SHOOTING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.flywheelsOn();
    if (shooter.isVelocitySet()) {
      intake.backFeedOn();
      LED.getInstance().changeLedState(LEDState.SHOT_FIRED);
    } else {
      intake.disableBackFeed();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.disableBackFeed();
    shooter.disableFlywheels();
    LED.getInstance().changeLedState(LEDState.IDLE);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (shooter.invShooterSensorOut()) {
      return true;
    }
    return false;
  }
}