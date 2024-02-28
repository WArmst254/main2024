package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LED.LEDState;
import frc.robot.subsystems.shooter.Shooter;
//import frc.robot.subsystems.vision.Vision;
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
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
  //private final Vision vision = new Vision();
  public static LED led = new LED();

//13 degrees
//26 inches

  // Controllers
  public static CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final CommandXboxController manualController = new CommandXboxController(2);

  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.WARNING);
      private final Alert manualDisconnected =
      new Alert("Manual controller disconnected (port 2).", AlertType.WARNING);

  // Auto Chooser
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

  private RotationSource summonRotation = new Joystick();
  // Use Initial Setpoints for Position Control
   void zeroSuperstructure() {
    elevator.zeroElevatorPosition();
    shooter.zeroShootAngle();
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

    NamedCommands.registerCommand("shoot", ShooterCommands.shootSensorCommand(shooter, intake, 0, 65));
    NamedCommands.registerCommand("shootOff", new InstantCommand(() -> shooter.disableShooter()).alongWith(new InstantCommand(() -> intake.disableBackFeed())));
    NamedCommands.registerCommand("intakeShooter", IntakeCommands.intakeToShooterSensorCommand(intake, shooter, 0.2));
    NamedCommands.registerCommand("elevatorUp", elevator.ampElevatorCommand());
    NamedCommands.registerCommand("elevatorDown", elevator.stowElevatorCommand());
    NamedCommands.registerCommand("amp", AmpCommands.ampAutonomousCommand(amp, elevator));
    NamedCommands.registerCommand("ampOff", new InstantCommand(() -> amp.disableAmp()));
    NamedCommands.registerCommand("intakeAmp", IntakeCommands.intakeToAmpSensorCommand(intake, amp));
    NamedCommands.registerCommand("intakeOff", new InstantCommand(() -> intake.disableIntake()));

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
    autoChooser.addOption("Starting AMP, Amp PL/Amp B1 (2)", new PathPlannerAuto("A2AmpPLAmpB1"));
    autoChooser.addOption("Starting SUB, Shoot PL/Shoot A2 (2)", new PathPlannerAuto("S2ShootPLShootA2"));
    autoChooser.addOption("Starting LONG, Shoot PL/Shoot A3 (2)", new PathPlannerAuto("L2ShootPLShootA3"));
    autoChooser.addOption("Starting LONG, Shoot PL/Shoot B5 (2)", new PathPlannerAuto("L2ShootPLShootB5"));
    autoChooser.addOption("Starting LONG, Shoot PL/Shoot A3/ShootA2 (3)", new PathPlannerAuto("L3ShootPLShootA3ShootA2"));
    autoChooser.addOption("Starting LONG, Shoot PL/Shoot A3/ShootA2 (4)", new PathPlannerAuto("L3ShootPLShootA3ShootA2ShootA1"));
  }

  public void configureButtonBindings() {
    double shooterCloseSpeed = 2500*Math.PI;

    // Drive Controls
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> summonRotation.getR()));

            
    Command intakeShooterAutoCommand = IntakeCommands.intakeToShooterSensorCommand(intake, shooter, 0.2);
    Command disableIntakeShooterAutoCommand = (new InstantCommand(() -> intake.disableIntake()).alongWith(new InstantCommand(() -> shooter.stowShootAngle())));
    Command outakeShooterAutoCommand = IntakeCommands.outakeFromShooterSensorCommand(intake, shooter);
    Command disableOutakeShooterAutoCommand = new InstantCommand(() -> intake.disableIntake()).alongWith(new InstantCommand(() -> shooter.disableShooter()));
    Command scoreShooterAutoCommand = ShooterCommands.shootSensorCommand(shooter, intake, 0, shooterCloseSpeed);
    Command disableScoreShooterAutoCommand = new InstantCommand(() -> shooter.disableShooter()).alongWith(new InstantCommand(() -> intake.disableBackFeed()));

    Command intakeAmpAutoCommand = IntakeCommands.intakeToAmpSensorCommand(intake, amp);
    Command disableIntakeAmpAutoCommand = new InstantCommand(() -> intake.disableIntake()).alongWith(new InstantCommand(() -> amp.disableAmp()));
    Command outakeAmpAutoCommand = IntakeCommands.outakeFromAmpSensorCommand(intake, amp);
    Command disableOutakeAmpAutoCommand = (new InstantCommand(() -> intake.disableIntake()).alongWith(new InstantCommand(() -> amp.disableAmp())));
    Command scoreAmpAutoCommand = new InstantCommand(() -> amp.ampOuttakeOn());
    Command disableScoreAmpAutoCommand =  new InstantCommand(() -> amp.disableAmp());
       

    Command intakeShooterManualCommand = new InstantCommand(() -> intake.intakeToShooter()).alongWith(new InstantCommand(() -> shooter.lowerShootAngle(0.18)));
    Command disableIntakeShooterManualCommand = new InstantCommand(() -> intake.disableIntake()).alongWith(new InstantCommand(() -> shooter.stowShootAngle()));
    Command outakeShooterManualCommand = new InstantCommand(() -> intake.outakeFromShooter()).alongWith(new InstantCommand(() -> shooter.lowerShootAngle(0.18)));
    Command disableOutakeShooterManualCommand = new InstantCommand(() -> intake.disableIntake()).alongWith(new InstantCommand(() -> shooter.stowShootAngle()));
    Command scoreShooterManualCommand = ShooterCommands.shootManualCommand(shooter, intake, 0, shooterCloseSpeed);
    Command disableScoreShooterManualCommand = new InstantCommand(() -> shooter.disableShooter()).alongWith(new InstantCommand(() -> intake.disableBackFeed()));
       
    Command intakeHPShooterCommand = new InstantCommand(() -> shooter.intakeHP());
    Command disableIntakeHPShooterCommand = new InstantCommand(() -> shooter.disableShooter());

    Command intakeAmpManualCommand = new InstantCommand(() -> intake.intakeToAmp()).alongWith(new InstantCommand(() -> amp.ampOuttakeOn()));
    Command disableIntakeAmpManualCommand = new InstantCommand(() -> intake.disableIntake()).alongWith(new InstantCommand(() -> amp.disableAmp()));
    Command outakeAmpManualCommand = new InstantCommand(() -> intake.outakeFromAmp());
    Command disableOutakeAmpManualCommand = new InstantCommand(() -> intake.disableIntake());
    Command scoreAmpManualCommand = new InstantCommand(() -> amp.ampOuttakeOn());
    Command disableScoreAmpManualCommand = new InstantCommand(() -> amp.disableAmp());
       
    Command intakeHPAmpCommand = ShooterCommands.HPintakeToAmpSensorCommand(intake, amp, shooter);
    Command disableIntakeHPAmpCommand = new InstantCommand(() -> shooter.disableShooter()).alongWith(new InstantCommand(() -> intake.disableFeeds())).alongWith(new InstantCommand(() -> amp.disableAmp()));

    //intake to shooter
    driverController
      .a()
      .whileTrue(intakeShooterAutoCommand);
    driverController
      .a()
      .onFalse(disableIntakeShooterAutoCommand);

    //intake to amp
    driverController
      .b()
      .whileTrue(intakeAmpAutoCommand);
    driverController
      .b()
      .onFalse(disableIntakeAmpAutoCommand);

    // score amp
    driverController
      .x()
      .whileTrue(scoreAmpAutoCommand);
    driverController
      .x()
      .onFalse(disableScoreAmpAutoCommand);
    
    // score shooter
    driverController
      .y()
      .whileTrue(scoreShooterAutoCommand);
    driverController
      .y()
      .onFalse(disableScoreShooterAutoCommand);


      //outake shooter
    driverController
      .rightTrigger()
      .whileTrue(outakeShooterAutoCommand);
    driverController
      .rightTrigger()
      .onFalse(disableOutakeShooterAutoCommand);

      //outake amp
    driverController
      .leftTrigger()
      .whileTrue(outakeAmpAutoCommand);
    driverController
      .leftTrigger()
      .onFalse(disableOutakeAmpAutoCommand);

    
      //manual intake to shooter
    manualController
      .a()
      .whileTrue(intakeShooterManualCommand);
    manualController
      .a()
      .onFalse(disableIntakeShooterManualCommand);

      //manual intake to amp
    manualController
      .b()
      .whileTrue(intakeAmpManualCommand);
    manualController
      .b()
      .onFalse(disableIntakeAmpManualCommand);

    // manual score amp
    manualController
      .x()
      .whileTrue(scoreAmpManualCommand);
    manualController
      .x()
      .onFalse(disableScoreAmpManualCommand);

      //manual score shooter
    manualController
      .y()
      .whileTrue(scoreShooterManualCommand);
    manualController
      .y()
      .onFalse(disableScoreShooterManualCommand);

      //manual outake shooter
    manualController
      .rightTrigger()
      .whileTrue(outakeShooterManualCommand);
    manualController
      .rightTrigger()
      .onFalse(disableOutakeShooterManualCommand);

    //manual outake amp
    manualController
      .leftTrigger()
      .whileTrue(outakeAmpManualCommand);
    manualController
      .leftTrigger()
      .onFalse(disableOutakeAmpManualCommand);

    // Human Player Shooter Intake
    operatorController
      .leftBumper()
      .whileTrue(intakeHPShooterCommand);
    operatorController
      .leftBumper()
      .onFalse(disableIntakeHPShooterCommand);

      // Human Player Amp Intake
    operatorController
      .rightBumper()
      .whileTrue(intakeHPAmpCommand);
    operatorController
      .rightBumper()
      .onFalse(disableIntakeHPAmpCommand);

    // Lower Shooter Angle
    driverController
      .leftBumper()
      .whileTrue(
        new InstantCommand(() -> shooter.lowerShootAngle(0.18)));

    // Stow(Raise) Shooter Angle
    driverController
      .leftBumper()
      .onFalse(
        new InstantCommand(() -> shooter.stowShootAngle()));

    // April Tag Lock
    driverController
    .rightBumper()
    .onTrue(new InstantCommand(() -> summonRotation = new AprilTagLock()));

    // Defer to Joystick
    driverController
    .rightBumper()
    .onFalse(new InstantCommand(() -> summonRotation = new Joystick()));

    // Extend Elevator to Amp Scoring Position
    driverController
      .povUp()
      .onTrue(
        new InstantCommand(() -> elevator.ampExtendElevator()));

    // Stow(Detract) Elevator
    driverController
      .povDown()
      .onTrue(
        new InstantCommand(() -> elevator.stowElevator()));

    // Reset Stowed Position
    driverController
      .povRight()
      .onTrue(
        new InstantCommand(() -> elevator.setElevatorStowPosition()));

    // Zero Gyro Yaw
    driverController
      .start()
      .onTrue(
        new InstantCommand(() -> gyro.zeroGyro()));

    // driverController //TODO: TEST INTERPOLATE METHOD
    //   .back()
    //   .whileTrue(
    //     ShooterCommands.interpolatedShootManualCommand(shooter, intake, shooter.getAutomaticState(vision)));
      
    // driverController
    //   .back()
    //   .onFalse(
    //     new InstantCommand(() -> shooter.disableShooter()));

    operatorController
      .rightTrigger()
      .whileTrue(
        new InstantCommand(() -> LED.getInstance().changeLedState(LEDState.HP_AMPLIFY)));
    operatorController
      .leftTrigger()
      .whileTrue(
        new InstantCommand(() -> LED.getInstance().changeLedState(LEDState.HP_COOPERTITION)));
  }


  public void checkControllers() {
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(driverController.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(driverController.getHID().getPort()));
    operatorDisconnected.set(
        !DriverStation.isJoystickConnected(operatorController.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(operatorController.getHID().getPort()));
    manualDisconnected.set(
        !DriverStation.isJoystickConnected(manualController.getHID().getPort())
              || !DriverStation.getJoystickIsXbox(manualController.getHID().getPort()));
  }

  public void checkSensors() {
    boolean intakeSensor = intake.intakeSensorOut();
    double intakeSensorRange = intake.intakeSensor();
    SmartDashboard.putBoolean("Intake Sensor: ", intakeSensor);
    SmartDashboard.putNumber("Intake Sensor Range: ", intakeSensorRange);
    
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
    shooter.stowShootAngle();
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
