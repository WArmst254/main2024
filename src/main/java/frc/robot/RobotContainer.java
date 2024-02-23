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
import frc.robot.Constants.TeleOpMode;
import frc.robot.commands.AmpCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
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
    shootAngle.stowShootAngle();
  }

  public void zeroSuperstructure() {
    elevator.zeroElevatorPosition();
    shootAngle.zeroShootAngle();
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

    NamedCommands.registerCommand("shoot", ShooterCommands.shootSensorCommand(shooter));
    NamedCommands.registerCommand("shootOff", new InstantCommand(() -> shooter.disableShooter()));
    NamedCommands.registerCommand(
        "intakeShooter", IntakeCommands.intakeToShooterSensorCommand(intake, shooter, shootAngle));
    NamedCommands.registerCommand("elevatorUp", elevator.ampElevatorCommand());
    NamedCommands.registerCommand("elevatorDown", elevator.stowElevatorCommand());
    NamedCommands.registerCommand("amp", AmpCommands.ampAutonomousCommand(amp, elevator));
    NamedCommands.registerCommand("ampOff", new InstantCommand(() -> amp.disableAmp()));
    NamedCommands.registerCommand(
        "intakeAmp", IntakeCommands.intakeToAmpSensorCommand(intake, amp));
    NamedCommands.registerCommand("intakeOff", new InstantCommand(() -> intake.disableIntake()));

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
    TeleOpMode mode = TeleOpMode.MANUAL_SPEAKER;

    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));

    if (operatorConditions.getYButtonPressed()) {
      mode = TeleOpMode.AUTO_SPEAKER;
    }
    if (operatorConditions.getAButtonPressed()) {
      mode = TeleOpMode.AUTO_AMP;
    }
    if (operatorConditions.getXButtonPressed()) {
      mode = TeleOpMode.MANUAL_SPEAKER;
    }
    if (operatorConditions.getBButtonPressed()) {
      mode = TeleOpMode.MANUAL_AMP;
    }

    switch (mode) {
      case AUTO_SPEAKER:
        driverController
            .a()
            .whileTrue(IntakeCommands.intakeToShooterSensorCommand(intake, shooter, shootAngle));
        driverController.a().onFalse(new InstantCommand(() -> intake.disableIntake()));
        driverController.x().whileTrue(IntakeCommands.outakeFromShooterSensorCommand(intake));
        driverController.x().onFalse(new InstantCommand(() -> intake.disableIntake()));
        driverController.y().whileTrue(ShooterCommands.shootSensorCommand(shooter));
        driverController.y().onFalse(new InstantCommand(() -> shooter.disableShooter()));
        break;
      case AUTO_AMP:
        driverController.a().whileTrue(IntakeCommands.intakeToAmpSensorCommand(intake, amp));
        driverController.a().onFalse(new InstantCommand(() -> intake.disableIntake()));
        driverController.x().whileTrue(IntakeCommands.outakeFromAmpSensorCommand(intake));
        driverController.x().onFalse(new InstantCommand(() -> intake.disableIntake()));
        driverController.y().whileTrue(AmpCommands.ampTeleopCommand(amp));
        driverController.y().onFalse(new InstantCommand(() -> amp.disableAmp()));
        break;
      case MANUAL_SPEAKER:
        driverController
            .a()
            .whileTrue(
                new InstantCommand(() -> intake.intakeToShooter())
                    .alongWith(new InstantCommand(() -> shootAngle.lowerShootAngle())));
        driverController
            .a()
            .onFalse(
                new InstantCommand(() -> intake.disableIntake())
                    .alongWith(new InstantCommand(() -> shootAngle.stowShootAngle())));
        driverController
            .x()
            .whileTrue(
                new InstantCommand(() -> intake.outakeFromShooter())
                    .alongWith(new InstantCommand(() -> shootAngle.lowerShootAngle())));
        driverController
            .x()
            .onFalse(
                new InstantCommand(() -> intake.disableIntake())
                    .alongWith(new InstantCommand(() -> shootAngle.stowShootAngle())));
        driverController.y().whileTrue(ShooterCommands.shootManualCommand(shooter));
        driverController.y().onFalse(new InstantCommand(() -> shooter.disableShooter()));
        break;
      case MANUAL_AMP:
        driverController
            .a()
            .whileTrue(
                new InstantCommand(() -> intake.intakeToAmp())
                    .alongWith(new InstantCommand(() -> amp.ampOuttakeOn())));
        driverController
            .a()
            .onFalse(
                new InstantCommand(() -> intake.disableIntake())
                    .alongWith(new InstantCommand(() -> amp.disableAmp())));
        driverController.x().whileTrue(new InstantCommand(() -> intake.outakeFromAmp()));
        driverController.x().onFalse(new InstantCommand(() -> intake.disableIntake()));
        driverController.y().whileTrue(new InstantCommand(() -> amp.ampOuttakeOn()));
        driverController.y().onFalse(new InstantCommand(() -> amp.disableAmp()));
        break;
    }

    driverController.leftBumper().whileTrue(new InstantCommand(() -> shootAngle.lowerShootAngle()));
    driverController.leftBumper().onFalse(new InstantCommand(() -> shootAngle.stowShootAngle()));

    operatorController.leftBumper().whileTrue(new InstantCommand(() -> shooter.intakeHP()));
    operatorController.leftBumper().onFalse(new InstantCommand(() -> shooter.disableShooter()));

    driverController.povUp().onTrue(new InstantCommand(() -> elevator.ampExtendElevator()));
    driverController.povDown().onTrue(new InstantCommand(() -> elevator.stowElevator()));
    driverController
        .povRight()
        .onTrue(new InstantCommand(() -> elevator.setElevatorStowPosition()));

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
    double ampSensorRange = amp.ampSensor();
    SmartDashboard.putNumber("amp sensor range", ampSensorRange);
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
