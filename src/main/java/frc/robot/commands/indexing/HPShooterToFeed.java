package frc.robot.commands.indexing;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED.LEDState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.intake.Intake;

public class HPShooterToFeed extends Command {

    private Intake intake;
    private Shooter shooter;
  
    /** Creates a new HPShooterFeed */
    public HPShooterToFeed(Intake intake, Shooter shooter) {
  
      this.intake = intake;
      this.shooter = shooter;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(intake, shooter);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      shooter.stowShooter();
       LED.getInstance().changeLedState(LEDState.INTAKING_SPEAKER);
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

      if(intake.intakeSensorOut()) {
        LED.getInstance().changeLedState(LEDState.NOTE_IN_FEED);
        } else {
          if (!shooter.shooterSensorOut()) {
            shooter.intakeHP();
          } else {
            shooter.disableFlywheels();
            shooter.lowerToIntake();
            intake.feedFromShooter();
          }
        }


  
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      intake.disableIntake();
      shooter.stowShooter();
      shooter.disableFlywheels();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      if (intake.intakeSensorOut()) {
        return true;
      }
      return false;
    }
  }
  
