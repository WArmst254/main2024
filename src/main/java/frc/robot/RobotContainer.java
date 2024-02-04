package frc.robot;

import com.ctre.phoenix6.hardware.TalonFX;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  private final SendableChooser<Command> autoChooser;

  // Controller
  private final CommandXboxController controller = new CommandXboxController(0);
  private final XboxController driverController = new XboxController(0);
  // private final XboxController operatorController = new XboxController(1);

  /* Controllers */
  // private final Joystick driver = new Joystick(0);

  // Dashboard inputs
  private final TalonFX intake = new TalonFX(62);

  private void intakeOnCommand() {
    intake.set(0.95);
  }

  private void intakeOffCommand() {
    intake.set(0);
  }

  private void outtakeOnCommand() {
    intake.set(-0.95);
  }

  private void outtakeOffCommand() {
    intake.set(0);
  }

  private final TalonFX shooterFeed = new TalonFX(60);
  private final CANSparkFlex shooterLeft =
      new CANSparkFlex(20, CANSparkLowLevel.MotorType.kBrushless);
  private final CANSparkFlex shooterRight =
      new CANSparkFlex(19, CANSparkLowLevel.MotorType.kBrushless);

  private void shooterOnCommand() {
    shooterFeed.set(-1);
    shooterLeft.set(-1);
    shooterRight.set(-.95);
  }

  private void shooterInCommand() {
    shooterFeed.set(1);
    shooterLeft.set(1);
    shooterRight.set(.95);
  }

  private void shooterOffCommand() {
    shooterFeed.set(0);
    shooterLeft.set(0);
    shooterRight.set(0);
  }

  private final TalonFX wrist = new TalonFX(17);

  private void wristUpOnCommand() {
    wrist.set(0.2);
  }

  private void wristUpOffCommand() {
    wrist.set(0);
  }

  private void wristDownOnCommand() {
    wrist.set(-0.2);
  }

  private void wristDownOffCommand() {
    wrist.set(0);
  }

  private final TalonFX shooterAngle = new TalonFX(15);

  private void shooterUpOnCommand() {
    shooterAngle.set(0.05);
  }

  private void shooterUpOffCommand() {
    shooterAngle.set(0);
  }

  private void shooterDownOnCommand() {
    shooterAngle.set(-0.05);
  }

  private void shooterDownOffCommand() {
    shooterAngle.set(0);
  }

  private final TalonFX elevator = new TalonFX(16);

  private void elevatorUpCommand() {
    elevator.set(0.05);
  }

  private void elevatorDownCommand() {
    shooterAngle.set(0);
  }

  private final TalonFX AmpLeft = new TalonFX(50);
  private final TalonFX AmpRight = new TalonFX(51);

  private void AmpOuttakeOnCommand() {
    AmpLeft.set(-.5);
    AmpRight.set(-.5);
  }

  private void AmpOuttakeOffCommand() {
    AmpLeft.set(0);
    AmpRight.set(0);
  }

  private void HPIntakeOnCommand() {
    AmpLeft.set(.5);
    AmpRight.set(-.5);
  }

  private void HPIntakeOffCommand() {
    AmpLeft.set(0);
    AmpRight.set(0);
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
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -controller.getLeftY(),
            () -> -controller.getLeftX(),
            () -> -controller.getRightX()));
    new Trigger(driverController::getAButton)
        .whileTrue(new InstantCommand(() -> intakeOnCommand()));

    new Trigger(driverController::getAButtonReleased)
        .onTrue(new InstantCommand(() -> intakeOffCommand()));

    new Trigger(driverController::getYButton)
        .whileTrue(new InstantCommand(() -> outtakeOnCommand()));

    new Trigger(driverController::getYButtonReleased)
        .onTrue(new InstantCommand(() -> outtakeOffCommand()));

    new Trigger(driverController::getXButton)
        .whileTrue(new InstantCommand(() -> shooterOnCommand()));
    new Trigger(driverController::getBButton)
        .whileTrue(new InstantCommand(() -> shooterInCommand()));

    new Trigger(driverController::getXButtonReleased)
        .onTrue(new InstantCommand(() -> shooterOffCommand()));
    new Trigger(driverController::getBButtonReleased)
        .onTrue(new InstantCommand(() -> shooterOffCommand()));

    new Trigger(driverController::getLeftBumper)
        .whileTrue(new InstantCommand(() -> wristDownOnCommand()));

    new Trigger(driverController::getLeftBumperReleased)
        .onTrue(new InstantCommand(() -> wristDownOffCommand()));

    new Trigger(driverController::getRightBumper)
        .whileTrue(new InstantCommand(() -> wristUpOnCommand()));

    new Trigger(driverController::getRightBumperReleased)
        .whileTrue(new InstantCommand(() -> wristUpOffCommand()));

    new Trigger(() -> driverController.getLeftTriggerAxis() > 0.1)
        .whileTrue(new InstantCommand(() -> shooterDownOnCommand()));

    new Trigger(() -> driverController.getLeftTriggerAxis() < 0.1)
        .onTrue(new InstantCommand(() -> shooterDownOffCommand()));

    new Trigger(() -> driverController.getRightTriggerAxis() > 0.1)
        .whileTrue(new InstantCommand(() -> shooterUpOnCommand()));

    new Trigger(() -> driverController.getRightTriggerAxis() < 0.1)
        .onTrue(new InstantCommand(() -> shooterUpOffCommand()));
    new Trigger(driverController::getStartButton)
        .onTrue(new InstantCommand(() -> HPIntakeOnCommand()));

    new Trigger(driverController::getStartButtonReleased)
        .onTrue(new InstantCommand(() -> HPIntakeOffCommand()));
    new Trigger(driverController::getBackButton)
        .onTrue(new InstantCommand(() -> AmpOuttakeOnCommand()));
    new Trigger(driverController::getBackButtonReleased)
        .onTrue(new InstantCommand(() -> AmpOuttakeOffCommand()));

    new Trigger(() -> driverController.getPOV() == 90)
        .onTrue(new InstantCommand(() -> elevatorUpCommand()));
    new Trigger(() -> driverController.getPOV() == 180)
        .onTrue(new InstantCommand(() -> elevatorDownCommand()));

    // Add a button to run the example auto to SmartDashboard, this will also be in the auto chooser
    // built above
    SmartDashboard.putData("Example Auto", new PathPlannerAuto("Example Auto"));

    // Add a button to SmartDashboard that will create and follow an on-the-fly path
    // This example will simply move the robot 2m in the +X field direction

    // 50 pos intake
    // 51 negative intake
    // both negative for outtake
  }

  public Command getAutonomousCommand() {
    // Load the path you want to follow using its name in the GUI
    PathPlannerPath path = PathPlannerPath.fromPathFile("StraightPath");
    return new PathPlannerAuto("StraightAuto");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
