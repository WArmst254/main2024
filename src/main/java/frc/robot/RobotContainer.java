package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AmpAuto;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeAuto;
import frc.robot.commands.ShootAuto;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shootangle.ShootAngle;
import frc.robot.subsystems.shooter.Shooter;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;
  private final Shooter shooter = new Shooter();
  private final Amp amp = new Amp();
  private final Intake intake = new Intake();
  private final Elevator elevator = new Elevator();
  private final GyroIOPigeon2 gyro = new GyroIOPigeon2(true);
  private final ShootAngle shootAngle = new ShootAngle();
  //Auto Chooser
  private final SendableChooser<Command> autoChooser;
  // Controllers
  private final CommandXboxController controller = new CommandXboxController(0);
  private final XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);

  private void zeroSwerveGyro() {
    gyro.zeroGyro();
  }
  
  private void setElevatorPos() {
    elevator.setElevatorPosition();
  }

  private void homeElevatorPos() {
    elevator.homeElevator();
  }

  private void setHomeposEle() {
    elevator.setElevatorHomePosition();
  }

  private void setShootAnglePos() {
    shootAngle.setShootAnglePosition();
  }

  private void homeShootAnglePos() {
    shootAngle.homeShootAngle();
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations

        drive =
            new Drive(
                new GyroIOPigeon2(true),
                new ModuleIOTalonFX(0),
                new ModuleIOTalonFX(1),
                new ModuleIOTalonFX(2),
                new ModuleIOTalonFX(3));
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim(),
                new ModuleIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        break;
    }
    NamedCommands.registerCommand("shoot", ShootAuto.shootAuto());
    NamedCommands.registerCommand("shootoff", shooter.disableShooter());
    NamedCommands.registerCommand("intakeshoot", IntakeAuto.intakeShootAuto());
    NamedCommands.registerCommand("elevatorup", elevator.autoAmpElevator());
    NamedCommands.registerCommand("elevatordown", elevator.autoHomeElevator());
    NamedCommands.registerCommand("amp", AmpAuto.ampAuto());
    NamedCommands.registerCommand("ampoff", amp.disableAmp());
    NamedCommands.registerCommand("intakeamp", IntakeAuto.intakeAmpAuto());
    NamedCommands.registerCommand("intakeoff", intake.disableIntake());
    // Configure the button bindings
    configureButtonBindings();
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`

    SmartDashboard.putData("Auto Mode", autoChooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Command intakeCommand = IntakeAuto.intakeShootAuto();
    Command outakeCommand = IntakeAuto.outakeShootAuto();
    Command scoringCommand = ShootAuto.shootAuto();
    Command disableCommand = shooter.disableShooter();

    elevator.zeroElevatorPosition();
    shootAngle.zeroShootAnglePosition();

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    if(operatorController.getYButtonPressed()) {
      intakeCommand = IntakeAuto.intakeShootAuto();
      outakeCommand = IntakeAuto.outakeShootAuto();
      scoringCommand = ShootAuto.shootAuto();
      disableCommand = shooter.disableShooter();
    }

    if(operatorController.getBButtonPressed()) {
      intakeCommand = IntakeAuto.intakeAmpAuto();
      outakeCommand = IntakeAuto.outakeAmpAuto();
      scoringCommand = AmpAuto.ampAuto();
      disableCommand = amp.disableAmp();
    }

    new Trigger(driverController::getAButton)
        .whileTrue(intakeCommand);
    new Trigger(driverController::getAButtonReleased)
        .onTrue(new InstantCommand(() -> intake.disableIntake()));

    new Trigger(driverController::getYButton)
        .whileTrue(outakeCommand);
    new Trigger(driverController::getYButtonReleased)
        .onTrue(new InstantCommand(() -> intake.disableIntake()));

    new Trigger(driverController::getBButton)
        .whileTrue(scoringCommand);
    new Trigger(driverController::getBButtonReleased)
        .onTrue(disableCommand);

    new Trigger(driverController::getLeftBumper)
        .whileTrue(new InstantCommand(() -> setShootAnglePos()));
    new Trigger(driverController::getLeftBumperReleased)
        .onTrue(new InstantCommand(() -> homeShootAnglePos()));

    new Trigger(() -> driverController.getPOV() == 0)
        .onTrue(new InstantCommand(() -> setElevatorPos()));
    new Trigger(() -> driverController.getPOV() == 90)
        .onTrue(new InstantCommand(() -> homeElevatorPos()));
    new Trigger(() -> driverController.getPOV() == 270)
        .onTrue(new InstantCommand(() -> setHomeposEle()));

    new Trigger(() -> driverController.getBackButton() || driverController.getStartButton())
        .onTrue(new InstantCommand(() -> zeroSwerveGyro()));
  }

  public Command getAutonomousCommand() {

    return new PathPlannerAuto("TwoPieceAmpB1");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
