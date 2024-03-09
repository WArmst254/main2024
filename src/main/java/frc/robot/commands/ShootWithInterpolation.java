// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED.LEDState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.intake.Intake;

public class ShootWithInterpolation extends Command {

  private Intake intake;
  private Shooter shooter;
  private Vision vision;

  /** Creates a new IntakeNote. */
  public ShootWithInterpolation(Intake intake, Shooter shooter, Vision vision) {

    this.intake = intake;
    this.shooter = shooter;
    this.vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, shooter, vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //shooter.lowerToShoot();
     LED.getInstance().changeLedState(LEDState.SUBWOOFER_SHOOTING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.interpolatedFlywheelVelocity(shooter.getAutomaticState(vision));
    shooter.interpolatedShooterAngle(shooter.getAutomaticState(vision));
    SmartDashboard.putNumber("ABC", shooter.getAutomaticState(vision).speed);
    if (shooter.isInterpolatedVelocitySet(shooter.getAutomaticState(vision)) && shooter.isAngleSet()) {
      intake.backFeedOn();
      LED.getInstance().changeLedState(LEDState.SHOT_FIRED);
    } else {
      intake.disableFeeds();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.disableFeeds();
    shooter.disableFlywheels();
    shooter.stowShooter();
    LED.getInstance().changeLedState(LEDState.IDLE);
  }

 // Returns true when the command should end.
  // @Override
  // public boolean isFinished() {
  //   if (shooter.invShooterSensorOut()) {
  //     return true;
  //   }
  //   return false;
  // }
}