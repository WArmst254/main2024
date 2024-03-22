package frc.robot;

import java.util.function.BooleanSupplier;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import static edu.wpi.first.wpilibj2.command.Commands.runOnce;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import frc.robot.commands.*;
import frc.robot.commands.indexing.BackfeedToScore;
import frc.robot.commands.indexing.FeedToShooter;
import frc.robot.commands.indexing.HPShooterToFeed;
import frc.robot.commands.indexing.VerySlowFeed;
import frc.robot.subsystems.amp.Amp;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED.LEDState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.AprilTagLock;

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
  private final Shooter shooter = new Shooter();
  private final Amp amp = new Amp();
  private final Intake intake = new Intake();
  private final Elevator elevator = new Elevator();
  private final GyroIOPigeon2 gyro = new GyroIOPigeon2(true);
  private final Vision vision = new Vision();
  public static LED led = new LED();


  // Auto Chooser
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

  private boolean speakerMode = true;
  private boolean ampMode = false;
  private boolean manualMode = false;

  private BooleanSupplier speakerModeSupplier = () -> speakerMode;
  private BooleanSupplier ampModeSupplier = () -> ampMode;

  // Controllers
  public static CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private Trigger runSpeakerSensorIntake = (driverController.a().or(operatorController.rightBumper())).and(() -> speakerModeSupplier.getAsBoolean());
  private Trigger runAmpSensorIntake = (driverController.a().or(operatorController.rightBumper())).and(ampModeSupplier);
  private Trigger podiumShot = driverController.b().and(ampModeSupplier);
  private Trigger subwooferShot = driverController.x().and(speakerModeSupplier);
  private Trigger interpolatedShot = driverController.y().and(speakerModeSupplier);
  private Trigger scoreAmp = driverController.y().and(ampModeSupplier);
  private Trigger revFlywheels = operatorController.rightTrigger().or(operatorController.leftTrigger());
  private Trigger resetGyro = driverController.start().or(driverController.back());
  private Trigger runSpeakerOuttake = operatorController.leftBumper().and(speakerModeSupplier);
  private Trigger runAmpOuttake = operatorController.leftBumper().and(ampModeSupplier);
  private Trigger selectSpeakerMode = operatorController.y();
  private Trigger selectAmpMode = operatorController.a();
  private Trigger extendElevatorToAmp = operatorController.pov(0);
  private Trigger retractElevator = operatorController.pov(180);
  private Trigger runSensorHPIntakeToSpeaker = operatorController.x().and(speakerModeSupplier);

  // Use Initial Setpoints for Position Control
  void zeroSuperstructure() {
    elevator.zeroElevatorPosition();
    shooter.zeroShooter();
  }

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

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

    NamedCommands.registerCommand("shootSub", new ShootFromSubwoofer(intake, shooter));
    NamedCommands.registerCommand("shootPod", new ShootFromPodium(intake, shooter));
    NamedCommands.registerCommand("shootInt", new ShootWithInterpolation(intake, shooter, vision));
    NamedCommands.registerCommand("shootOff", runOnce(() -> shooter.disableFlywheels()).alongWith(runOnce(() -> intake.disableFeeds())));
    NamedCommands.registerCommand("intakeShooter", new GroundIntakeToShooter(intake, shooter).andThen(new BackfeedToScore(intake, shooter)).andThen(new VerySlowFeed(intake, shooter)));
    NamedCommands.registerCommand("elevatorUp", elevator.ampElevatorCommand());
    NamedCommands.registerCommand("elevatorDown", elevator.stowElevatorCommand());
    NamedCommands.registerCommand("amp", new ScoreAmp(intake, amp, elevator));
    NamedCommands.registerCommand("ampOff", runOnce(() -> amp.disableAmp()).alongWith(elevator.stowElevatorCommand()));
    NamedCommands.registerCommand("intakeAmp", new GroundIntakeToAmp(intake, amp, elevator));
    NamedCommands.registerCommand("intakeOff", runOnce(() -> intake.disableIntake()).alongWith(runOnce(() -> shooter.stowShooter())).alongWith(runOnce(() -> amp.disableAmp())));

    zeroSuperstructure();
    configureAutos();
    configureButtonBindings();
  }

  private void configureAutos() {
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Old 2 Amp", new PathPlannerAuto("A2AmpPLAmpB1"));
    autoChooser.addOption("Old 2 Sub", new PathPlannerAuto("S2ShootPLShootA2"));
    autoChooser.addOption("Old 2 Long", new PathPlannerAuto("L2ShootPLShootB5"));
    autoChooser.addOption("Old 4 Long", new PathPlannerAuto("L4ShootPLShootA3ShootA2ShootA1"));

    autoChooser.addOption("New 2 Amp", new PathPlannerAuto("2 Note Amp"));
    autoChooser.addOption("New 3 Sub-Mid", new PathPlannerAuto("3 Note Center-Center"));
    autoChooser.addOption("New 2 Long", new PathPlannerAuto("2 Note Long"));
    autoChooser.addOption("New 4 Long", new PathPlannerAuto("4 Note Long"));

  }

  public void configureButtonBindings() {

    selectSpeakerMode.onTrue(runOnce(() -> {
      speakerMode = true;
      ampMode = false;
       LED.getInstance().changeLedState(LEDState.SHOOTER_ABSENT);
    }));

    selectAmpMode.onTrue(runOnce(() -> {
      speakerMode = false;
      ampMode = true;
      LED.getInstance().changeLedState(LEDState.AMP_ABSENT);
    }));

    // Drive Controls
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> ((driverController.y().getAsBoolean() || driverController.rightBumper().getAsBoolean()) && speakerMode) ? AprilTagLock.getR() : -driverController.getRightX()));

    resetGyro
      .onTrue(
        runOnce(() -> gyro.zeroGyro()));

    runSpeakerSensorIntake
      .toggleOnTrue(
        new GroundIntakeToShooter(intake, shooter)
        .andThen(new BackfeedToScore(intake, shooter))
        .andThen(new VerySlowFeed(intake, shooter)))
      .onFalse(
        runOnce(() -> intake.disableIntake())
        .alongWith(
          runOnce(() -> shooter.stowShooter())));

    runAmpSensorIntake
      .toggleOnTrue(
        new GroundIntakeToAmp(intake, amp, elevator))
      .onFalse(
        runOnce(() -> intake.disableIntake())
        .alongWith(
          runOnce(() -> amp.disableAmp())));

    scoreAmp
      .whileTrue(
        runOnce(() -> amp.ampOuttakeOn()))
      .onFalse(
          runOnce(() -> amp.disableAmp()));

    subwooferShot
      .whileTrue(
        new ShootFromSubwoofer(intake, shooter))
      .onFalse(
        runOnce(() -> intake.disableFeeds())
        .alongWith(
          runOnce(() -> shooter.disableFlywheels())));
    
          podiumShot
          .whileTrue(
            new ShootFromPodium(intake, shooter))
          .onFalse(
            runOnce(() -> intake.disableFeeds())
            .alongWith(
              runOnce(() -> shooter.disableFlywheels())));

    interpolatedShot
      .whileTrue(
        new ShootWithInterpolation(intake, shooter, vision))
      .onFalse(
        runOnce(() -> intake.disableFeeds())
        .alongWith(
          runOnce(() -> shooter.disableFlywheels())));

    extendElevatorToAmp
      .onTrue(
        runOnce(() -> elevator.ampExtendElevator()));

    retractElevator
      .onTrue(
        runOnce(() -> elevator.stowElevator()));

    runAmpOuttake
      .whileTrue(
        runOnce(() -> amp.ampIntakeOn())
        .alongWith(
          runOnce(() -> intake.outakeFromAmp())))
        .onFalse(
          runOnce(() -> intake.disableIntake())
          .alongWith(
            runOnce(() -> amp.disableAmp())));

    runSpeakerOuttake
      .whileTrue(
        runOnce(() -> shooter.intakeHP())
        .alongWith(
          runOnce(() -> intake.outakeFromShooter())))
      .onFalse(
        runOnce(() -> shooter.disableFlywheels())
        .alongWith(
          runOnce(() -> intake.disableIntake())));

    revFlywheels
      .whileTrue(
        new RevFlywheels(shooter, vision))
        .onFalse(
          runOnce(() -> shooter.disableFlywheels()));

    runSensorHPIntakeToSpeaker
      .toggleOnTrue(
        new HPShooterToFeed(intake, shooter)
        .andThen(
          new FeedToShooter(intake, shooter)))
      .onFalse(
        runOnce(() -> shooter.disableFlywheels())
        .alongWith(
          runOnce(() -> intake.disableIntake())));
  }

  public void checkSensors() {

    SmartDashboard.putBoolean("Speaker Mode", speakerMode);
    SmartDashboard.putBoolean("Amp Mode", ampMode);
    SmartDashboard.putBoolean("Manual Mode", manualMode);

    boolean intakeSensor = intake.intakeSensorOut();
    double intakeSensorRange = intake.intakeSensor();
    SmartDashboard.putBoolean("Intake Sensor: ", intakeSensor);
    SmartDashboard.putNumber("Intake Sensor Range: ", intakeSensorRange);

    double LLdistance = vision.getDistance();
    SmartDashboard.putNumber("Limelight Range: ", LLdistance);

    boolean shooterSensor = shooter.shooterSensorOut();
    double shooterSensorRange = shooter.shooterSensor();
    SmartDashboard.putBoolean("Shooter Sensor: ", shooterSensor);
    SmartDashboard.putNumber("Shooter Sensor Range: ", shooterSensorRange);

    boolean ampSensor = amp.ampSensorOut();
    double ampSensorRange = amp.ampSensor();
    SmartDashboard.putBoolean("Amp Sensor: ", ampSensor);
    SmartDashboard.putNumber("Amp Sensor Range: ", ampSensorRange);

  }

  public void stowShooterAngle() {
    shooter.stowShooter();
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

}