package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED.LEDState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.intake.Intake;

public class HPSpeakerRefeed extends Command {

    private Intake intake;
    private Shooter shooter;
  
    /** Creates a new IntakeNote. */
    public HPSpeakerRefeed(Intake intake, Shooter shooter) {
  
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
        
      if (shooter.invShooterSensorOut()) {
        shooter.intakeHP();
      } else {
        shooter.lowerToIntake();
        intake.feedFromShooter();
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
  
