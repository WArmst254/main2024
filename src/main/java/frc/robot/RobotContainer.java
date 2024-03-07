package frc.robot;

import com.ctre.phoenix.led.FireAnimation;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AmpCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.IntakeNoteAmp;
import frc.robot.commands.IntakeNoteSpeaker;
import frc.robot.commands.ShootFromSubwoofer;
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
import frc.robot.util.Alert;
import frc.robot.util.Alert.AlertType;
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

  private double shooterFeedAngle = 0.2;
  private double shooterScoreAngle = 0;
  private double shooterScoreSpeed = 2500*Math.PI;

  // Auto Chooser
  private final LoggedDashboardChooser<Command> autoChooser = new LoggedDashboardChooser<>("Auto Choices");

  private boolean speakerMode = true;
  private boolean ampMode = false;
  private boolean manualMode = false;

  // Controllers
  public static XboxController driverController = new XboxController(0);
  private final XboxController operatorController = new XboxController(1);

  private Trigger runSpeakerSensorIntake = new Trigger(() -> driverController.getAButton() && (speakerMode && !manualMode));
  private Trigger runAmpSensorIntake = new Trigger(() -> driverController.getAButton() && (ampMode && !manualMode));
  private Trigger runSpeakerManualIntake = new Trigger(() -> driverController.getAButton() && (speakerMode && manualMode));
  private Trigger runAmpManualIntake = new Trigger(() -> driverController.getAButton() && (ampMode && manualMode));

  private Trigger subwooferShot = new Trigger(() -> driverController.getXButton() && speakerMode);
  private Trigger podiumShot = new Trigger(() -> driverController.getBButton() && speakerMode);
  private Trigger shootWithInterpolation = new Trigger(() -> driverController.getYButton() && speakerMode);

  private Trigger scoreAmp = new Trigger(() -> driverController.getYButton() && ampMode);

  private Trigger alignScoreSpeaker = new Trigger(() -> driverController.getRightBumper() && speakerMode);
  private Trigger alignScoreAmp = new Trigger(() -> driverController.getRightBumper() && ampMode);

  private Trigger resetGyro = new Trigger(() -> driverController.getStartButton());

  private Trigger runSpeakerOuttake = new Trigger(() -> operatorController.getRightTriggerAxis() > 0.1 && speakerMode);
  private Trigger runAmpOuttake = new Trigger(() -> operatorController.getRightTriggerAxis() > 0.1 && ampMode);

  private Trigger selectSpeakerMode = new Trigger(() -> operatorController.getYButton());
  private Trigger selectAmpMode = new Trigger(() -> operatorController.getAButton());
  private Trigger selectManualMode = new Trigger(() -> operatorController.getBackButton());

  private Trigger aprilTagLock = new Trigger(() -> driverController.getRightBumper());

  private RotationSource summonRotation = new Joystick();
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

    NamedCommands.registerCommand("shoot", ShooterCommands.shootSensorCommand(shooter, intake, shooterScoreAngle, shooterScoreSpeed));
    NamedCommands.registerCommand("shootOff", new InstantCommand(() -> shooter.disableFlywheels()).alongWith(new InstantCommand(() -> intake.disableBackFeed())));
    NamedCommands.registerCommand("intakeShooter", IntakeCommands.intakeToShooterSensorCommand(intake, shooter, shooterFeedAngle));
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
   // autoChooser.addOption("Starting LONG, Shoot PL/Shoot A3/ShootA2 (3)", new PathPlannerAuto("L3ShootPLShootA3ShootA2"));
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
      if(speakerMode==true && ampMode==false) {
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
    runSpeakerSensorIntake.whileTrue(new IntakeNoteSpeaker(intake, shooter)).onFalse(new InstantCommand(() -> intake.disableIntake()).alongWith(new InstantCommand(() -> shooter.stowShooter())));
    runAmpSensorIntake.whileTrue(new IntakeNoteAmp(intake, amp, elevator)).onFalse(new InstantCommand(() -> intake.disableIntake()).alongWith(new InstantCommand(() -> amp.disableAmp())));
    scoreAmp.whileTrue(new InstantCommand(() -> amp.ampOuttakeOn())).onFalse(new InstantCommand(() -> amp.disableAmp()).alongWith(new InstantCommand(() -> LED.getInstance().changeLedState(LEDState.IDLE))));
    subwooferShot.whileTrue(new ShootFromSubwoofer(intake, shooter)).onFalse(new InstantCommand(() -> intake.disableBackFeed()).alongWith(new InstantCommand(() -> shooter.disableFlywheels())));
    //aprilTagLock.whileTrue(new InstantCommand(() -> summonRotation = new AprilTagLock())).onFalse(new InstantCommand(() -> summonRotation = new Joystick()));
    // Command outakeShooterAutoCommand = IntakeCommands.outakeFromShooterSensorCommand(intake, shooter);
    // Command disableOutakeShooterAutoCommand = new InstantCommand(() -> intake.disableIntake()).alongWith(new InstantCommand(() -> shooter.disableFlywheels()));

    // Command outakeAmpAutoCommand = IntakeCommands.outakeFromAmpSensorCommand(intake, amp);
    // Command disableOutakeAmpAutoCommand = (new InstantCommand(() -> intake.disableIntake()).alongWith(new InstantCommand(() -> amp.disableAmp())));
    // Command scoreAmpAutoCommand = new InstantCommand(() -> amp.ampOuttakeOn());
    // Command disableScoreAmpAutoCommand =  new InstantCommand(() -> amp.disableAmp());

    // Command intakeShooterManualCommand = new InstantCommand(() -> intake.intakeToShooter()).alongWith(new InstantCommand(() -> shooter.lowerShooter(shooterFeedAngle)));
    // Command disableIntakeShooterManualCommand = new InstantCommand(() -> intake.disableIntake()).alongWith(new InstantCommand(() -> shooter.stowShooter()));
    // Command outakeShooterManualCommand = new InstantCommand(() -> intake.outakeFromShooter()).alongWith(new InstantCommand(() -> shooter.lowerShooter(shooterFeedAngle)));
    // Command disableOutakeShooterManualCommand = new InstantCommand(() -> intake.disableIntake()).alongWith(new InstantCommand(() -> shooter.stowShooter()));
    // Command scoreShooterManualCommand = ShooterCommands.shootManualCommand(shooter, intake, shooterScoreAngle, shooterScoreSpeed);
    // Command disableScoreShooterManualCommand = new InstantCommand(() -> shooter.disableFlywheels()).alongWith(new InstantCommand(() -> intake.disableBackFeed()));
       
    // Command intakeHPShooterCommand = new InstantCommand(() -> shooter.intakeHP());
    // Command disableIntakeHPShooterCommand = new InstantCommand(() -> shooter.disableFlywheels());

    // Command intakeAmpManualCommand = new InstantCommand(() -> intake.intakeToAmp()).alongWith(new InstantCommand(() -> amp.ampOuttakeOn()));
    // Command disableIntakeAmpManualCommand = new InstantCommand(() -> intake.disableIntake()).alongWith(new InstantCommand(() -> amp.disableAmp()));
    // Command outakeAmpManualCommand = new InstantCommand(() -> intake.outakeFromAmp());
    // Command disableOutakeAmpManualCommand = new InstantCommand(() -> intake.disableIntake());
    // Command scoreAmpManualCommand = new InstantCommand(() -> amp.ampOuttakeOn());
    // Command disableScoreAmpManualCommand = new InstantCommand(() -> amp.disableAmp());
       
    // Command intakeHPAmpCommand = ShooterCommands.HPintakeToAmpSensorCommand(intake, amp, shooter);
    // Command disableIntakeHPAmpCommand = new InstantCommand(() -> shooter.disableFlywheels()).alongWith(new InstantCommand(() -> intake.disableFeeds())).alongWith(new InstantCommand(() -> amp.disableAmp()));

    // //intake to shooter
    // driverController
    //   .a()
    //   .whileTrue(intakeShooterAutoCommand);
    // driverController
    //   .a()
    //   .onFalse(disableIntakeShooterAutoCommand);

    // //intake to amp
    // operatorController
    //   .b()
    //   .whileTrue(intakeAmpAutoCommand);
    // operatorController
    //   .b()
    //   .onFalse(disableIntakeAmpAutoCommand);

    // // score amp
    // driverController
    //   .x()
    //   .whileTrue(scoreAmpAutoCommand);
    // driverController
    //   .x()
    //   .onFalse(disableScoreAmpAutoCommand);
    
    // // score shooter
    // driverController
    //   .y()
    //   .whileTrue(scoreShooterAutoCommand);
    // driverController
    //   .y()
    //   .onFalse(disableScoreShooterAutoCommand);


    //   //outake shooter
    // operatorController
    //   .rightTrigger()
    //   .whileTrue(outakeShooterAutoCommand);
    // operatorController
    //   .rightTrigger()
    //   .onFalse(disableOutakeShooterAutoCommand);

    //   //outake amp
    // operatorController
    //   .leftTrigger()
    //   .whileTrue(outakeAmpAutoCommand);
    // operatorController
    //   .leftTrigger()
    //   .onFalse(disableOutakeAmpAutoCommand);

    // // Human Player Shooter Intake
    // operatorController
    //   .leftBumper()
    //   .whileTrue(intakeHPShooterCommand);
    // operatorController
    //   .leftBumper()
    //   .onFalse(disableIntakeHPShooterCommand);

    //   // Human Player Amp Intake
    // operatorController
    //   .rightBumper()
    //   .whileTrue(intakeHPAmpCommand);
    // operatorController
    //   .rightBumper()
    //   .onFalse(disableIntakeHPAmpCommand);

    // // Lower Shooter Angle
    // driverController
    //   .leftBumper()
    //   .whileTrue(
    //     new InstantCommand(() -> shooter.lowerShooter(0.15)));

    // // Stow(Raise) Shooter Angle
    // driverController
    //   .leftBumper()
    //   .onFalse(
    //     new InstantCommand(() -> shooter.stowShooter()));


    // // Extend Elevator to Amp Scoring Position
    // operatorController
    //   .povUp()
    //   .onTrue(
    //     new InstantCommand(() -> elevator.ampExtendElevator()));

    // // Stow(Detract) Elevator
    // operatorController
    //   .povDown()
    //   .onTrue(
    //     new InstantCommand(() -> elevator.stowElevator()));

    // // Reset Stowed Position
    // operatorController
    //   .povRight()
    //   .onTrue(
    //     new InstantCommand(() -> elevator.setElevatorStowPosition()));

    // operatorController
    //   .start()
    //   .whileTrue(
    //     new InstantCommand(() -> shooter.flywheelsOn(shooterScoreSpeed))
    //   );
    //   operatorController
    //   .start()
    //   .onFalse(
    //     new InstantCommand(() -> shooter.disableFlywheels())
    //   );

    //   operatorController
    //   .back()
    //   .whileTrue(
    //     new InstantCommand(() -> shooter.flywheelsOn(shooterScoreSpeed))
    //   );
    //   operatorController
    //   .back()
    //   .onFalse(
    //     new InstantCommand(() -> shooter.disableFlywheels())
    //   );

    // // Zero Gyro Yaw
    // driverController
    //   .start()
    //   .onTrue(
    //     );

    // // driverController //TODO: TEST INTERPOLATE METHOD
    // //   .back()
    // //   .whileTrue(
    // //     ShooterCommands.interpolatedShootManualCommand(shooter, intake, shooter.getAutomaticState(vision)));
      
    // // driverController
    // //   .back()
    // //   .onFalse(
    // //     new InstantCommand(() -> shooter.disableShooter()));

    // operatorController
    //   .a()
    //   .whileTrue(
    //     new InstantCommand(() -> LED.getInstance().changeLedState(LEDState.HP_AMPLIFY)));
    // operatorController
    //   .y()
    //   .whileTrue(
    //     new InstantCommand(() -> LED.getInstance().changeLedState(LEDState.HP_COOPERTITION)));
    //    operatorController
    //     .a()
    //     .onFalse(
    //       new InstantCommand(() -> LED.getInstance().changeLedState(LEDState.IDLE)));
    //   operatorController
    //     .y()
    //     .onFalse(
    //       new InstantCommand(() -> LED.getInstance().changeLedState(LEDState.IDLE)));
      }


  // public void checkControllers() {
  //   driverDisconnected.set(
  //       !DriverStation.isJoystickConnected(driverController.getHID().getPort())
  //           || !DriverStation.getJoystickIsXbox(driverController.getHID().getPort()));
  //   operatorDisconnected.set(
  //       !DriverStation.isJoystickConnected(operatorController.getHID().getPort())
  //           || !DriverStation.getJoystickIsXbox(operatorController.getHID().getPort()));
  // }

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
