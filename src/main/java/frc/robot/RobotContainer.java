package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

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

  // Controllers
  private final XboxController operatorConditions = new XboxController(1);
  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.INFO);

  // Auto Chooser
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  public void homeShootAnglePosition() {
    shootAngle.homeShootAngle();
  }

  public void zeroSuperstructure() {
    elevator.zeroElevatorPosition();
    shootAngle.zeroShootAnglePosition();
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

    NamedCommands.registerCommand("shoot", ShootAuto.shootAuto(shooter));
    NamedCommands.registerCommand("shootOff", shooter.disableShooter());
    NamedCommands.registerCommand("intakeShooter", IntakeAuto.intakeShootAuto(intake, shooter));
    NamedCommands.registerCommand("elevatorUp", elevator.autoAmpElevator());
    NamedCommands.registerCommand("elevatorDown", elevator.autoHomeElevator());
    NamedCommands.registerCommand("amp", AmpAuto.ampAuto(amp, elevator));
    NamedCommands.registerCommand("ampOff", amp.disableAmp());
    NamedCommands.registerCommand("intakeAmp", IntakeAuto.intakeAmpAuto(intake, amp));
    NamedCommands.registerCommand("intakeOff", intake.disableIntake());

    zeroSuperstructure();
    configureAutos();
    configureButtonBindings();
  }

  private void configureAutos() {
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Two Piece A2 Starting Sub", new PathPlannerAuto("2ShootSubA2"));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Command intakeCommand = IntakeAuto.intakeShootAuto(intake, shooter);
    Command outakeCommand = IntakeAuto.outakeShootAuto(intake);
    Command scoringCommand = ShootAuto.shootAuto(shooter);
    Command disableCommand = shooter.disableShooter();

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    if (operatorConditions.getYButtonPressed()) {
      intakeCommand = IntakeAuto.intakeShootAuto(intake, shooter);
      outakeCommand = IntakeAuto.outakeShootAuto(intake);
      scoringCommand = ShootAuto.shootAuto(shooter);
      disableCommand = shooter.disableShooter();
    }

    if (operatorConditions.getBButtonPressed()) {
      intakeCommand = IntakeAuto.intakeAmpAuto(intake, amp);                      
      outakeCommand = IntakeAuto.outakeAmpAuto(intake);
      scoringCommand = AmpAuto.ampTele(amp);
      disableCommand = amp.disableAmp();
    }

    driverController.a().whileTrue(intakeCommand);
    driverController.a().onFalse(new InstantCommand(() -> intake.intakeOff()));

    driverController.y().whileTrue(outakeCommand);
    driverController.y().onFalse(new InstantCommand(() -> intake.disableIntake()));

    driverController.b().whileTrue(scoringCommand);
    driverController.b().onFalse(disableCommand);

    driverController
        .leftBumper()
        .whileTrue(new InstantCommand(() -> shootAngle.setShootAnglePosition()));
    driverController.leftBumper().onFalse(new InstantCommand(() -> shootAngle.homeShootAngle()));

    driverController.povUp().onTrue(new InstantCommand(() -> elevator.setElevatorPosition()));
    driverController.povDown().onTrue(new InstantCommand(() -> elevator.homeElevator()));
    driverController
        .povRight()
        .onTrue(new InstantCommand(() -> elevator.setElevatorHomePosition()));

    driverController.start().onTrue(new InstantCommand(() -> gyro.zeroGyro()));
  }

  public void checkControllers() {
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(driverController.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(driverController.getHID().getPort()));
    operatorDisconnected.set(
        !DriverStation.isJoystickConnected(operatorController.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(operatorController.getHID().getPort()));
  }

  public void checkSensors() {
    boolean intakeSensor = intake.intakeSensorOut();
    SmartDashboard.putBoolean("intake sensor", intakeSensor);
    boolean shooterSensor = shooter.shooterSensorOut();
    SmartDashboard.putBoolean("shooter sensor", shooterSensor);
    boolean ampSensor = amp.ampSensorOut();
    SmartDashboard.putBoolean("amp sensor", ampSensor);
  }

  public Command getAutonomousCommand() {

    return new PathPlannerAuto("2ShootSubA2");
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
