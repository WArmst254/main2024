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

  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);
  private final Alert operatorDisconnected =
      new Alert("Operator controller disconnected (port 1).", AlertType.WARNING);

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
    autoChooser.addOption("Exit Amp", new PathPlannerAuto("ExitAmp"));
    autoChooser.addOption("Exit Sub", new PathPlannerAuto("ExitSub"));
    autoChooser.addOption("Exit Long", new PathPlannerAuto("ExitLong"));
    autoChooser.addOption("Amp Both Two Piece (A1) Starting Amp", new PathPlannerAuto("2AmpA1"));
    autoChooser.addOption("Amp Both Two Piece (B1) Starting Amp", new PathPlannerAuto("2AmpB1"));
    autoChooser.addOption("Shoot Both Two Piece (A3) Starting Long", new PathPlannerAuto("2ShootLongA3"));
    autoChooser.addOption("Shoot Both Two Piece (B5) Starting Long", new PathPlannerAuto("2ShootLongB5"));
    autoChooser.addOption("Shoot Both Two Piece (A2) Starting Sub", new PathPlannerAuto("2ShootSubA2"));
    autoChooser.addOption("Amp All Three Piece (B1,A1) Starting Amp", new PathPlannerAuto("3AmpB1AmpA1"));
    autoChooser.addOption("Amp Two & Shoot One Three Piece (B1,A1) Starting Amp", new PathPlannerAuto("3AmpB1ShootA1"));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  public void configureButtonBindings() {
    // Default TeleOperated Mode -- Sensor-Enabled Shooter Scoring Mode
  

    // Drive Controls
    drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            drive,
            () -> -driverController.getLeftY(),
            () -> -driverController.getLeftX(),
            () -> summonRotation.getR()));

            
      Command intakeShooterAutoCommand = IntakeCommands.intakeToShooterSensorCommand(intake, shooter, 0.2);
      Command disableIntakeShooterAutoCommand = new InstantCommand(() -> intake.disableIntake());
       Command outakeShooterAutoCommand = IntakeCommands.outakeFromShooterSensorCommand(intake, shooter);
       Command disableOutakeShooterAutoCommand = new InstantCommand(() -> intake.disableIntake()).alongWith(new InstantCommand(() -> shooter.disableShooter()));
       Command scoreShooterAutoCommand = ShooterCommands.shootSensorCommand(shooter, intake, 0, 50);
       Command disableScoreShooterAutoCommand = new InstantCommand(() -> shooter.disableShooter()).alongWith(new InstantCommand(() -> intake.disableBackFeed()));

       Command intakeAmpAutoCommand = IntakeCommands.intakeToAmpSensorCommand(intake, amp);
       Command disableIntakeAmpAutoCommand = new InstantCommand(() -> intake.disableIntake()).alongWith(new InstantCommand(() -> amp.disableAmp()));
       Command outakeAmpAutoCommand = IntakeCommands.outakeFromAmpSensorCommand(intake, amp);
       Command disableOutakeAmpAutoCommand = new InstantCommand(() -> intake.disableIntake());
       Command scoreAmpAutoCommand = new InstantCommand(() -> amp.ampOuttakeOn());
       Command disableScoreAmpAutoCommand =  new InstantCommand(() -> amp.disableAmp());
       

      Command intakeShooterManualCommand = new InstantCommand(() -> intake.intakeToShooter()).alongWith(new InstantCommand(() -> shooter.lowerShootAngle(0.18)));
      Command disableIntakeShooterManualCommand = new InstantCommand(() -> intake.disableIntake());
      Command outakeShooterManualCommand = new InstantCommand(() -> intake.outakeFromShooter()).alongWith(new InstantCommand(() -> shooter.lowerShootAngle(0.18)));
       Command disableOutakeShooterManualCommand = new InstantCommand(() -> intake.disableIntake()).alongWith(new InstantCommand(() -> shooter.stowShootAngle()));
       Command scoreShooterManualCommand = ShooterCommands.shootManualCommand(shooter, intake, 0, 65);
       Command disableScoreShooterManualCommand = new InstantCommand(() -> shooter.disableShooter()).alongWith(new InstantCommand(() -> intake.disableBackFeed()));
       
       Command intakeHPShooterCommand = new InstantCommand(() -> shooter.intakeHP());
       Command disableIntakeHPShooterCommand = new InstantCommand(() -> shooter.disableShooter());

       Command intakeAmpManualCommand = new InstantCommand(() -> intake.intakeToAmp()).alongWith(new InstantCommand(() -> amp.ampOuttakeOn()));
       Command disableIntakeAmpManualCommand = new InstantCommand(() -> intake.disableIntake()).alongWith(new InstantCommand(() -> amp.disableAmp()));
       Command outakeAmpManualCommand = new InstantCommand(() -> intake.outakeFromAmp());
       Command disableOutakeAmpManualCommand = new InstantCommand(() -> intake.disableIntake());
       Command scoreAmpManualCommand = new InstantCommand(() -> amp.ampOuttakeOn());
       Command disableScoreAmpManualCommand = new InstantCommand(() -> amp.disableAmp());
       
       Command intakeHPAmpCommand = new InstantCommand(() -> shooter.intakeHP()).alongWith(new InstantCommand(() -> intake.feedHPIntakeToAmp())).alongWith(new InstantCommand(() -> amp.ampOuttakeOn()));
       Command disableIntakeHPAmpCommand = new InstantCommand(() -> shooter.disableShooter()).alongWith(new InstantCommand(() -> intake.disableFeeds())).alongWith(new InstantCommand(() -> amp.disableAmp()));

    
    driverController
      .a()
      .whileTrue(intakeShooterAutoCommand);
    driverController
      .a()
      .onFalse(disableIntakeShooterAutoCommand);

      driverController
      .b()
      .whileTrue(intakeAmpAutoCommand);
    driverController
      .a()
      .onFalse(disableIntakeAmpAutoCommand);

    // Ground Outtake
    driverController
      .x()
      .whileTrue(outakeShooterAutoCommand);
    driverController
      .x()
      .onFalse(disableOutakeShooterAutoCommand);
    
    // Shooter Scoring
    driverController
      .y()
      .whileTrue(outakeAmpAutoCommand);
    driverController
      .y()
      .onFalse(disableOutakeAmpAutoCommand);

      driverController
      .rightTrigger()
      .whileTrue(scoreShooterAutoCommand);
    driverController
      .rightTrigger()
      .onFalse(disableScoreShooterAutoCommand);

      driverController
      .leftTrigger()
      .whileTrue(scoreAmpAutoCommand);
    driverController
      .leftTrigger()
      .onFalse(disableScoreAmpAutoCommand);

      operatorController
      .a()
      .whileTrue(intakeShooterManualCommand);
    operatorController
      .a()
      .onFalse(disableIntakeShooterManualCommand);

      operatorController
      .b()
      .whileTrue(intakeAmpManualCommand);
    operatorController
      .a()
      .onFalse(disableIntakeAmpManualCommand);

    // Ground Outtake
    operatorController
      .x()
      .whileTrue(outakeShooterManualCommand);
    operatorController
      .x()
      .onFalse(disableOutakeShooterManualCommand);
    
    // Shooter Scoring
    operatorController
      .y()
      .whileTrue(outakeAmpManualCommand);
    operatorController
      .y()
      .onFalse(disableOutakeAmpManualCommand);

      operatorController
      .rightTrigger()
      .whileTrue(scoreShooterManualCommand);
    operatorController
      .rightTrigger()
      .onFalse(disableScoreShooterManualCommand);

      operatorController
      .leftTrigger()
      .whileTrue(scoreAmpManualCommand);
    operatorController
      .leftTrigger()
      .onFalse(disableScoreAmpManualCommand);


    // Human Player Intake
    operatorController
      .leftBumper()
      .whileTrue(intakeHPShooterCommand);
    operatorController
      .leftBumper()
      .onFalse(disableIntakeHPShooterCommand);

      // Human Player Intake
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
      .povRight()
      .whileTrue(
        new InstantCommand(() -> LED.getInstance().changeLedState(LEDState.HP_AMPLIFY)));
    operatorController
      .povLeft()
      .whileTrue(
        new InstantCommand(() -> LED.getInstance().changeLedState(LEDState.HP_COOPERTITION)));
  }

  public void teleopPeriodic() {

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
