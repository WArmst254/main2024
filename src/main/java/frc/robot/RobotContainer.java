package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AmpCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.FeedToAmp;
import frc.robot.commands.FeedToSpeaker;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.GroundIntakeToAmp;
import frc.robot.commands.GroundIntakeToSpeaker;
import frc.robot.commands.HPToAmpFeed;
import frc.robot.commands.HPSpeakerRefeed;
import frc.robot.commands.ShootFromSubwoofer;
import frc.robot.commands.ShootWithInterpolation;
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
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED.LEDState;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.vision.Vision;
import frc.robot.util.rotation.AprilTagLock;
import frc.robot.util.rotation.Joystick;
import frc.robot.util.rotation.RotationSource;

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
  private final Vision vision = new Vision();
  public static LED led = new LED();

  // Auto Chooser
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

  private boolean speakerMode = true;
  private boolean ampMode = false;
  private boolean manualMode = false;

  private RotationSource summonRotation = new Joystick();

  // Controllers
  public static XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);

  private Trigger runSpeakerSensorIntake = new Trigger(() -> driverController.getAButton() && (speakerMode && !manualMode));
  private Trigger runAmpSensorIntake = new Trigger(() -> driverController.getAButton() && (ampMode && !manualMode));
  
  private Trigger runSpeakerManualIntake = new Trigger(() -> driverController.getAButton() && (speakerMode && manualMode));
  private Trigger runAmpManualIntake = new Trigger(() -> driverController.getAButton() && (ampMode && manualMode));
  
  private Trigger subwooferShot = new Trigger(() -> driverController.getXButton() && speakerMode);
  private Trigger podiumShot = new Trigger(() -> driverController.getBButton() && speakerMode);
  private Trigger interpolatedShot = new Trigger(() -> driverController.getYButton() && speakerMode);
  private Trigger scoreAmp = new Trigger(() -> driverController.getYButton() && ampMode);

  //private Trigger alignScoreAmp = new Trigger(() -> driverController.getRightBumper() && ampMode);
  private Trigger aprilTagLock  = new Trigger(() -> driverController.getRightBumper() && speakerMode || driverController.getYButton() && speakerMode);

  private Trigger resetGyro = new Trigger(() -> driverController.getStartButton());

  private Trigger runSpeakerOuttake = new Trigger(() -> operatorController.getRightTriggerAxis() > 0.1 && speakerMode);
  private Trigger runAmpOuttake = new Trigger(() -> operatorController.getRightTriggerAxis() > 0.1 && ampMode);

  private Trigger selectSpeakerMode = new Trigger(() -> operatorController.getYButton());
  private Trigger selectAmpMode = new Trigger(() -> operatorController.getAButton());
  private Trigger selectManualMode = new Trigger(() -> operatorController.getBackButton() || operatorController.getStartButton());

  private Trigger extendElevatorToAmp = new Trigger(() -> operatorController.getPOV() == 0);
  private Trigger retractElevator = new Trigger(() ->  operatorController.getPOV() == 180);

  private Trigger runSensorHPIntakeToAmp = new Trigger(() -> operatorController.getXButton() && ampMode && !manualMode);
  private Trigger runSensorHPIntakeToSpeaker = new Trigger(() -> operatorController.getXButton() && speakerMode && !manualMode);
  

  // Use Initial Setpoints for Position Control
   void zeroSuperstructure() {
    elevator.zeroElevatorPosition();
    shooter.zeroShooter();
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

    NamedCommands.registerCommand("shoot", ShooterCommands.shootSensorCommand(shooter, intake, 0, 3500));
    NamedCommands.registerCommand("shootOff", new InstantCommand(() -> shooter.disableFlywheels()).alongWith(new InstantCommand(() -> intake.disableFeeds())));
    NamedCommands.registerCommand("intakeShooter", IntakeCommands.intakeToShooterSensorCommand(intake, shooter, 0.2));
    NamedCommands.registerCommand("elevatorUp", elevator.ampElevatorCommand());
    NamedCommands.registerCommand("elevatorDown", elevator.stowElevatorCommand());
    NamedCommands.registerCommand("amp", AmpCommands.ampTeleopSensorCommand(amp));
    NamedCommands.registerCommand("ampOff", new InstantCommand(() -> amp.disableAmp()).alongWith(elevator.stowElevatorCommand()));
    NamedCommands.registerCommand("ampoff", new InstantCommand(() -> amp.disableAmp()).alongWith(elevator.stowElevatorCommand()));
    NamedCommands.registerCommand("intakeAmp", IntakeCommands.intakeToAmpSensorCommand(intake, amp));
    NamedCommands.registerCommand("intakeOff", new InstantCommand(() -> intake.disableIntake()).alongWith(new InstantCommand(() -> shooter.stowShooter())).alongWith(new InstantCommand(() -> amp.disableAmp())));

    zeroSuperstructure();
    configureAutos();
    configureButtonBindings();
  }

  private void configureAutos() {
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption("Exit AMP", new PathPlannerAuto("A0Exit"));
    autoChooser.addOption("Exit SUB", new PathPlannerAuto("S0Exit"));
    autoChooser.addOption("Exit LONG", new PathPlannerAuto("L0Exit"));
    autoChooser.addOption("Starting AMP, Amp PL/Exit (1)", new PathPlannerAuto("A1AmpPL"));
    autoChooser.addOption("Starting SUB, Shoot PL/Exit (1)", new PathPlannerAuto("S1ShootPL"));
    autoChooser.addOption("Starting LONG, Shoot PL/Exit (1)", new PathPlannerAuto("L1ShootPL"));
    autoChooser.addOption("Starting AMP, Amp PL/Amp A1 (2)", new PathPlannerAuto("A2AmpPLAmpA1"));
    autoChooser.addOption("2AMP", new PathPlannerAuto("A2AmpPLAmpB1"));
    autoChooser.addOption("Starting SUB, Shoot PL/Shoot A2 (2)", new PathPlannerAuto("S2ShootPLShootA2"));
    autoChooser.addOption("Starting LONG, Shoot PL/Shoot A3 (2)", new PathPlannerAuto("L2ShootPLShootA3"));
    autoChooser.addOption("Starting LONG, Shoot PL/Shoot B5 (2)", new PathPlannerAuto("L2ShootPLShootB5"));
    autoChooser.addOption("Starting LONG, Shoot PL/Shoot A3/ShootA2 (4)", new PathPlannerAuto("L4ShootPLShootA3ShootA2ShootA1"));
  }

  public void configureButtonBindings() {

    selectSpeakerMode.onTrue(new InstantCommand(() -> {
      speakerMode = true;
      ampMode = false;
      LED.getInstance().changeLedState(LEDState.SPEAKER);
    }));

    selectAmpMode.onTrue(new InstantCommand(() -> {
      speakerMode = false;
      ampMode = true;
      LED.getInstance().changeLedState(LEDState.AMP);
    }));

    selectManualMode.onTrue(new InstantCommand(() -> {
      if(speakerMode==false && ampMode==true) {
        speakerMode = false;
        ampMode = true;
        manualMode = true;
      } else {
        speakerMode = true;
        ampMode = false;
        manualMode = true;
      }
    }));
  
    // Drive Controls
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> summonRotation.getR()));

    resetGyro.onTrue(new InstantCommand(() -> gyro.zeroGyro()));
    runSpeakerSensorIntake.whileTrue(new GroundIntakeToSpeaker(intake, shooter)).onFalse(new InstantCommand(() -> intake.disableIntake()).alongWith(new InstantCommand(() -> shooter.stowShooter())));
    runAmpSensorIntake.whileTrue(new GroundIntakeToAmp(intake, amp, elevator)).onFalse(new InstantCommand(() -> intake.disableIntake()).alongWith(new InstantCommand(() -> amp.disableAmp())));
    scoreAmp.whileTrue(new InstantCommand(() -> amp.ampOuttakeOn())).onFalse(new InstantCommand(() -> amp.disableAmp()).alongWith(new InstantCommand(() -> LED.getInstance().changeLedState(LEDState.IDLE))));
    subwooferShot.whileTrue(new ShootFromSubwoofer(intake, shooter)).onFalse(new InstantCommand(() -> intake.disableFeeds()).alongWith(new InstantCommand(() -> shooter.disableFlywheels())));
    interpolatedShot.whileTrue(new ShootWithInterpolation(intake, shooter, vision)).onFalse(new InstantCommand(() -> intake.disableFeeds()).alongWith(new InstantCommand(() -> shooter.disableFlywheels())));
    aprilTagLock.whileTrue(new InstantCommand(() -> summonRotation = new AprilTagLock())).onFalse(new InstantCommand(() -> summonRotation = new Joystick()));
    extendElevatorToAmp.onTrue(new InstantCommand(() -> elevator.ampExtendElevator()));
    retractElevator.onTrue(new InstantCommand(() -> elevator.stowElevator()));
    runAmpOuttake.onTrue(new InstantCommand(() -> amp.ampIntakeOn()).alongWith(new InstantCommand(() -> intake.outakeFromAmp()))).onFalse(new InstantCommand(() -> intake.disableIntake()).alongWith(new InstantCommand(() -> amp.disableAmp())));
    runSpeakerOuttake.onTrue(new InstantCommand(() -> shooter.intakeHP()).alongWith(new InstantCommand(() -> intake.outakeFromShooter()))).onFalse(new InstantCommand(() -> shooter.disableFlywheels()).alongWith(new InstantCommand(() -> intake.disableIntake())));
    
    runSensorHPIntakeToAmp.onTrue(new HPToAmpFeed(intake, shooter).andThen(new FeedToAmp(intake, amp, elevator, shooter))).onFalse(new InstantCommand(() -> shooter.disableFlywheels()).alongWith(new InstantCommand(() -> intake.disableFeeds())).alongWith(new InstantCommand(() -> amp.disableAmp())));
    runSensorHPIntakeToSpeaker.onTrue(new HPSpeakerRefeed(intake, shooter).andThen(new FeedToSpeaker(intake, shooter))).onFalse(new InstantCommand(() -> shooter.disableFlywheels()).alongWith(new InstantCommand(() -> intake.disableFeeds())));
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

  public  Command getAutonomousCommand() {
    return autoChooser.get();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
}
