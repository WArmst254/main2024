// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED.LEDState;

public class ShootFromPodium extends Command {

  private Intake intake;
  private Shooter shooter;

  /** Creates a new ShootFromPodium */
  public ShootFromPodium(Intake intake, Shooter shooter) {

    this.intake = intake;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.lowerToShootPod();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(shooter.shooterSensorOut()) {
    shooter.flywheelsOnPod();
    if (shooter.isPodiumVelocitySet() && shooter.isAngleSet()) {
      intake.pushToShoot();
    } else {
      intake.disableFeeds();
    } } else {
      LED.getInstance().changeLedState(LEDState.SHOOTER_ABSENT);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.disableFeeds();
    shooter.disableFlywheels();
    shooter.stowShooter();
  }

 //Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (!shooter.shooterSensorOut()) {
      return true;
    }
    return false;
  }
}