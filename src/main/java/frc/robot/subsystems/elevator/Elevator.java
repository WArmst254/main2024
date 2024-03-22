package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TunableNumber;

public class Elevator extends SubsystemBase {

  private final TalonFX leadElevator = new TalonFX(Constants.IDs.leadElevator);
  private final TalonFX followerElevator = new TalonFX(Constants.IDs.followerElevator);
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0).withEnableFOC(true);

  private TunableNumber eVelocity = new TunableNumber("Elevator/Velocity");
  private TunableNumber eAcceleration = new TunableNumber("Elevator/Acceleration");
  private TunableNumber eJerk = new TunableNumber("Elevator/Jerk");
  private TunableNumber eTolerance = new TunableNumber("Elevator PID/Tolerance");
  private TunableNumber ampPosition = new TunableNumber("Elevator/Amp Setpoint");
  private TunableNumber eP = new TunableNumber("Elevator PID/P");
  private TunableNumber eI = new TunableNumber("Elevator PID/I");
  private TunableNumber eD = new TunableNumber("Elevator PID/D");
  private TunableNumber eV = new TunableNumber("Elevator PID/V");
  private TunableNumber eS = new TunableNumber("Elevator PID/S");

  public Elevator() {
    followerElevator.setControl(new Follower(Constants.IDs.leadElevator , false));    
    ampPosition.setDefault(ElevatorConstants.ampPosition);
    eTolerance.setDefault(ElevatorConstants.elevatorTolerance);
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    eVelocity.setDefault(ElevatorConstants.elevatorVelocity);
    eAcceleration.setDefault(ElevatorConstants.elevatorAcceleration);
    eJerk.setDefault(ElevatorConstants.elevatorJerk);

    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = eVelocity.get();
    mm.MotionMagicAcceleration = eAcceleration.get();
    mm.MotionMagicJerk = eJerk.get();
    cfg.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    eP.setDefault(ElevatorConstants.elevatorP);
    eI.setDefault(ElevatorConstants.elevatorI);
    eD.setDefault(ElevatorConstants.elevatorD);
    eV.setDefault(ElevatorConstants.elevatorV);
    eS.setDefault(ElevatorConstants.elevatorS);


    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = eP.get();
    slot0.kI = eI.get();
    slot0.kD = eD.get();
    slot0.kV = eV.get();
    slot0.kS = eS.get();

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 28;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = leadElevator.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }

  public void periodic() {
    SmartDashboard.putNumber("Elevator/Reported Position: ", leadElevator.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Reported Velocity: ", leadElevator.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Reported Power:", leadElevator.get());
    SmartDashboard.putNumber("Elevator/Reported Voltage:", leadElevator.getMotorVoltage().getValueAsDouble());
  }

  public void ampExtendElevator() {
    leadElevator.setControl(m_mmReq.withPosition(ElevatorConstants.ampPosition).withSlot(0));
  }

  public void stowElevator() {
    leadElevator.setControl(m_mmReq.withPosition(0).withSlot(0));
  }

  public void zeroElevatorPosition() {
    leadElevator.setPosition(0);
  }

  public boolean isElevatorSet() {
    return (m_mmReq.Position <= (leadElevator.getPosition().getValueAsDouble()+eTolerance.get()) && m_mmReq.Position >= (leadElevator.getPosition().getValueAsDouble()-eTolerance.get()));
  }

  public boolean isElevatorAmped() {
    return (ampPosition.get() <= (leadElevator.getPosition().getValueAsDouble()+eTolerance.get()) && ampPosition.get() >= (leadElevator.getPosition().getValueAsDouble()-eTolerance.get()));
  }

  public Command ampElevatorCommand() {
    return runOnce(
            () -> {
              leadElevator.setControl(m_mmReq.withPosition(ampPosition.get()).withSlot(0));
            })
        .andThen(run(() -> {}).withTimeout(0.05))
        .withName("Elevator Lifted");
  }

  public Command stowElevatorCommand() {
    return runOnce(
            () -> {
              leadElevator.setControl(m_mmReq.withPosition(0).withSlot(0));
            })
        .andThen(run(() -> {}).withTimeout(0.05))
        .withName("Elevator Stowed");
  }
}
