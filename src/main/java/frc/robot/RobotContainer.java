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
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AmpAutoOff;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeAutoOff;
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

  private final SendableChooser<Command> autoChooser;
  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final XboxController driverController = new XboxController(0);

  /* Controllers */

  // Dashboard inputs
  // private final TalonFX intake = new TalonFX(17);
  // private final TalonFX feedFront = new TalonFX(13);
  // private final TalonFX feedBack = new TalonFX(14);

  private void intakeOnCommand() {
    intake.autoIntake();
    // intakeon = new IntakeAutoOff();
    // intake.set(1);
    // feedFront.set(-0.95);
    // feedBack.set(0.95);
  }

  private void intakeOffCommand() {
    intake.intakeOff();
  }

  void outtakeOnCommand() {
    intake.outakeOnShoot();
    System.out.println("running");
  }

  private void outtakeOffCommand() {
    intake.intakeOff();
  }

  ShootAngle shootPid = new ShootAngle();

  // private final TalonFX AmpLeft = new TalonFX(18);
  // private final TalonFX AmpRight = new TalonFX(19);

  private void AmpOuttakeOnCommand() {
    amp.AmpOuttakeOn();
  }

  private void AmpOuttakeOffCommand() {
    amp.AmpOuttakeOff();
  }

  GyroIOPigeon2 gyro = new GyroIOPigeon2(true);

  private void zeroSwerveGyro() {
    gyro.zeroGyro();
  }

  Elevator elevator = new Elevator();

  private void setElevatorPos() {
    elevator.setElevatorPosition();
  }

  private void homeElevatorPos() {
    elevator.homeElevator();
  }

  private void setHomeposEle() {
    elevator.setHomePos();
  }

  // Shooter shooter = new Shooter();

  // private void autofeedShooter() {
  //   shooter.shootCommand(6000);
  // }

  ShootAngle shootAngle = new ShootAngle();

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
    NamedCommands.registerCommand("shoot", shooter.shootCommand(70));
    NamedCommands.registerCommand("shootoff", shooter.disableShooter());
    NamedCommands.registerCommand("intakeshoot", IntakeAutoOff.intakeShootAuto());
    NamedCommands.registerCommand("elevatorup", elevator.autoAmpElevator());
    NamedCommands.registerCommand("elevatordown", elevator.autoHomeElevator());
    NamedCommands.registerCommand("amp", AmpAutoOff.ampAuto());
    NamedCommands.registerCommand("ampoff", amp.disableAmp());
    NamedCommands.registerCommand("intakeamp", IntakeAutoOff.intakeAmpAuto());
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
    elevator.zeroElevatorPosition();
    shootAngle.zeroShootAnglePosition();

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));

    new Trigger(driverController::getAButton)
        .whileTrue(new RepeatCommand(IntakeAutoOff.intakeShootAuto()))
        .onFalse(new InstantCommand(() -> intakeOffCommand()));

    new Trigger(driverController::getYButton)
        .whileTrue(new InstantCommand(() -> outtakeOnCommand()));
    new Trigger(driverController::getYButtonReleased)
        .onTrue(new InstantCommand(() -> outtakeOffCommand()));

    // new Trigger(driverController::getXButton)
    //     .whileTrue(new InstantCommand(() -> shooterOnCommand()));
    // new Trigger(driverController::getXButtonReleased)
    //     .onTrue(new InstantCommand(() -> shooterOffCommand()));

    // new Trigger(driverController::getBButton)
    //     .whileTrue(new InstantCommand(() -> shooterInCommand()));
    // new Trigger(driverController::getBButtonReleased)
    //     .onTrue(new InstantCommand(() -> shooterOffCommand()));

    new Trigger(driverController::getLeftBumper)
        .whileTrue(new InstantCommand(() -> setShootAnglePos()));
    new Trigger(driverController::getLeftBumperReleased)
        .onTrue(new InstantCommand(() -> homeShootAnglePos()));

    new Trigger(driverController::getRightBumper).whileTrue(shooter.teleshootCommand(70));

    new Trigger(() -> driverController.getRightTriggerAxis() > 0.1)
        .whileTrue(new InstantCommand(() -> AmpOuttakeOnCommand()));
    new Trigger(() -> driverController.getRightTriggerAxis() <= 0.1)
        .onTrue(new InstantCommand(() -> AmpOuttakeOffCommand()));

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
    // Load the path you want to follow using its name in the GUI
    // PathPlannerPath path = PathPlannerPath.fromPathFile("StraightPath");

    return new PathPlannerAuto("TwoPieceAmpB1");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
