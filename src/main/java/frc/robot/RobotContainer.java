package frc.robot;

import java.util.function.BooleanSupplier;
import com.pathplanner.lib.commands.PathPlannerAuto;
import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.*;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // Subsystems
  private final Drive drive;
  private final Feeder feeder;
  private final Amp amp;

  // Auto Chooser
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

  private boolean speakerMode = true;
  private boolean ampMode = false;
  private boolean climbMode = false;
  private boolean manualMode = false;

  private boolean hasFeederNote = false;
  private boolean hasAmpNote = false;

  private BooleanSupplier speakerModeSupplier = () -> speakerMode;
  private BooleanSupplier ampModeSupplier = () -> ampMode;
  private BooleanSupplier climbModeSupplier = () -> climbMode;
  private BooleanSupplier manualModeSupplier = () -> manualMode;

  private BooleanSupplier hasFeederNoteSupplier = () -> hasFeederNote;
  private BooleanSupplier hasAmpNoteSupplier = () -> hasAmpNote;

  // Controllers
  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);

  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.WARNING);

  private Trigger resetGyro = driverController.start().or(driverController.back());

  private Trigger runShooterIntake = (operatorController.rightBumper()).and(speakerModeSupplier).and(manualModeSupplier).negate();
  private Trigger runAmpIntake = (operatorController.rightBumper()).and(ampModeSupplier).and(manualModeSupplier).negate();
    private Trigger manualOverrideRunShooterIntake = (operatorController.rightBumper()).and(speakerModeSupplier).and(manualModeSupplier);
    private Trigger manualOverrideRunAmpIntake = (operatorController.rightBumper()).and(ampModeSupplier).and(manualModeSupplier);

  private Trigger subwooferShot = driverController.x().and(speakerModeSupplier);
  private Trigger podiumShot = driverController.b().and(ampModeSupplier);

  private Trigger alignAmp = driverController.rightBumper().and(ampModeSupplier);
  private Trigger alignSpeaker = driverController.rightBumper().and(speakerModeSupplier);

  private Trigger InterpolationShootingSequence = operatorController.rightTrigger().or(operatorController.leftTrigger()).and(speakerModeSupplier);
  private Trigger feedToShoot = driverController.y().and(speakerModeSupplier);
    private Trigger incrementInterpolationAngle = operatorController.rightStick();
    private Trigger decremetInterpolationAngle = operatorController.leftStick();

  private Trigger scoreAmp = driverController.y().and(ampModeSupplier);

  private Trigger runShooterOuttake = operatorController.leftBumper().and(speakerModeSupplier).or(hasFeederNoteSupplier);
  private Trigger runAmpOuttake = operatorController.leftBumper().and(ampModeSupplier).or(hasAmpNoteSupplier);

  private Trigger indexShooterToAmp = operatorController.a().and(hasFeederNoteSupplier).and(manualModeSupplier).negate();
  private Trigger indexAmpToShooter = operatorController.y().and(hasAmpNoteSupplier).and(manualModeSupplier).negate();

  private Trigger selectSpeakerMode = operatorController.y();
  private Trigger selectAmpMode = operatorController.a();
  private Trigger selectClimbMode = operatorController.povLeft();
  private Trigger selectManualMode = operatorController.back().or(operatorController.start());

  private Trigger extendElevatorToAmp = operatorController.povUp().and(climbModeSupplier).negate();
  private Trigger extendElevatorToClimb = operatorController.povUp().and(climbModeSupplier);
  private Trigger stowElevator = operatorController.povDown();

  private Trigger runHPIntakeToSpeaker = operatorController.x().and(speakerModeSupplier).and(manualModeSupplier).negate();
  private Trigger runHPIntakeToAmp = operatorController.x().and(ampModeSupplier).and(manualModeSupplier).negate();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    feeder = new Feeder();
    amp = new Amp();
    switch (Constants.currentMode) {
      case REAL:
        // Real robot, instantiate hardware IO implementations
        drive = new Drive(
            new GyroIOPigeon2(true),
            new ModuleIOTalonFX(0),
            new ModuleIOTalonFX(1),
            new ModuleIOTalonFX(2),
            new ModuleIOTalonFX(3));
            Shuffleboard.getTab("Subsystems").add("Swerve", drive);
        break;

      case SIM:
        // Sim robot, instantiate physics sim IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim(),
            new ModuleIOSim());
        break;

      default:
        // Replayed robot, disable IO implementations
        drive = new Drive(
            new GyroIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            },
            new ModuleIO() {
            });
        break;
       
    }
    configureAutos();
    configureButtonBindings();
    hasAmpNote = amp.isNotePresentTOF();
    hasFeederNote = feeder.isNotePresentTOF();
  }

  private void configureAutos() {

    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Old 2 Amp", new PathPlannerAuto("A2AmpPLAmpB1"));
    autoChooser.addOption("Old 2 Sub", new PathPlannerAuto("S2ShootPLShootA2"));
    autoChooser.addOption("Old 2 Long", new PathPlannerAuto("L2ShootPLShootB5"));
    autoChooser.addOption("Old 4 Long", new PathPlannerAuto("L4ShootPLShootA3ShootA2ShootA1"));
    autoChooser.addOption("ampside", new PathPlannerAuto("ampside"));

    autoChooser.addOption("New 2 Amp", new PathPlannerAuto("2 Note Amp"));
    autoChooser.addOption("New 3 Sub-Mid", new PathPlannerAuto("3 Note Center-Center"));
    autoChooser.addOption("New 2 Long", new PathPlannerAuto("2 Note Long"));
    autoChooser.addOption("New 4 Long", new PathPlannerAuto("4 Note Long"));
  }

  public void configureButtonBindings() {

    selectSpeakerMode.onTrue(runOnce(() -> {
      speakerMode = true;
      ampMode = false;
    }));

    selectAmpMode.onTrue(runOnce(() -> {
      speakerMode = false;
      ampMode = true;
    }));

    selectClimbMode.toggleOnTrue(runOnce(() -> {
      climbMode = !climbMode;
    }));

    selectManualMode.toggleOnTrue(runOnce(() -> {
      manualMode = !manualMode;
    }));

    // Drive Controls
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> -driverController.getRightX()));
  }

  /** Updates the alerts for disconnected controllers. */
  public void checkControllers() {
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(driverController.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(driverController.getHID().getPort()));
    operatorDisconnected.set(
        !DriverStation.isJoystickConnected(operatorController.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(operatorController.getHID().getPort()));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}