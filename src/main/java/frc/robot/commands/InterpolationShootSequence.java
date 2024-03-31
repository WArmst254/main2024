package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.interpolate.ShooterInterpolation;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Shooter;

public class InterpolationShootSequence extends Command{
    private Pivot pivot;
    private Shooter shooter;
    private Feeder feeder;



    /** Creates a new Shoot. */
  public InterpolationShootSequence(Shooter shooter, Pivot pivot, Feeder feeder) {

    this.shooter = shooter;
    this.pivot = pivot;
    this.feeder = feeder;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, pivot, feeder);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double targetDistance = 5;

    pivot.setAngle(ShooterInterpolation.calculatePivotAngle(targetDistance));
    shooter.setVelocity(ShooterInterpolation.calculateShooterRPM(targetDistance));

    
  }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      shooter.stop();
      feeder.stop();
      pivot.stow();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
    
}
