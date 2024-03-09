package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED.LEDState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;

public class FeedToAmp extends Command {

  private Intake intake;
  private Amp amp;
  private Elevator elevator;
  private Shooter shooter;

  /** Creates a new IntakeNote. */
  public FeedToAmp(Intake intake, Amp amp, Elevator elevator, Shooter shooter) {

    this.intake = intake;
    this.amp = amp;
    this.elevator = elevator;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, amp, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.stowElevator();
    shooter.lowerToIntake();
    LED.getInstance().changeLedState(LEDState.INTAKING_AMP);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (amp.invAmpSensorOut() && !elevator.isElevatorSet()) {
      intake.disableIntake();
    } else if (amp.invAmpSensorOut() && elevator.isElevatorSet()) {
      intake.intakeToAmp();
      amp.ampOuttakeOn();
    } else {
      intake.disableIntake();
      LED.getInstance().changeLedState(LEDState.GROUND_INTAKE_DETECTED);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.disableIntake();
    amp.disableAmp();
    shooter.stowShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (amp.ampSensorOut()) {
      return true;
    }
    return false;
  }
}