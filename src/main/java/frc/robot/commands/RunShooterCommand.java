// package frc.robot.commands;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.IntakeSubsystem;

// public class RunShooterCommand extends Command {
//   /** Creates a new RunIntakeCommand. */

//   IntakeSubsystem m_IntakeSubsystem;
//   double m_speed;

//   public RunIntakeCommand(double speed, IntakeSubsystem IntakeSubsystem) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     m_IntakeSubsystem = IntakeSubsystem;
//     m_speed = speed;

//     addRequirements(IntakeSubsystem);

//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {

//     m_IntakeSubsystem.RunIntake(m_speed);

//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {}

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {

//     m_IntakeSubsystem.RunIntake(0);

//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
