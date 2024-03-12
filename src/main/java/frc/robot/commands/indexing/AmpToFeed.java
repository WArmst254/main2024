package frc.robot.commands.indexing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED.LEDState;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.intake.Intake;

public class AmpToFeed extends Command {

    private Intake intake;
    private Amp amp;
  
    /** Creates a new AmpToFeed. */
    public AmpToFeed(Intake intake, Amp amp) {
  
      this.intake = intake;
      this.amp = amp;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(intake, amp);
    }
  
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
       LED.getInstance().changeLedState(LEDState.INTAKING_SPEAKER);
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      if(intake.intakeSensorOut()) {
        LED.getInstance().changeLedState(LEDState.NOTE_IN_FEED);
      } else {
        amp.ampIntakeOn();
        intake.feedFromAmp();
        LED.getInstance().changeLedState(LEDState.INTAKING_SPEAKER);
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
      if (intake.intakeSensorOut()) {
        return true;
      }
      return false;
    }
  }
