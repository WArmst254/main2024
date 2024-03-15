package frc.robot.subsystems.elevator;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TunableNumber;

public class Elevator extends SubsystemBase {

  private final TalonFX elevator1 = new TalonFX(Constants.IDs.elevator1);
  private final TalonFX elevator2 = new TalonFX(Constants.IDs.elevator2);
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

  private TunableNumber mm_eVelocity = new TunableNumber("Elevator/Velocity");
  private TunableNumber mm_eAcceleration = new TunableNumber("Elevator/Acceleration");
  private TunableNumber mm_eJerk = new TunableNumber("Elevator/Jerk");
  private TunableNumber eTolerance = new TunableNumber("Elevator PID/Tolerance");
  private TunableNumber ampPosition = new TunableNumber("Elevator/Amp Setpoint");
  private TunableNumber eP = new TunableNumber("Elevator PID/P");
  private TunableNumber eI = new TunableNumber("Elevator PID/I");
  private TunableNumber eD = new TunableNumber("Elevator PID/D");
  private TunableNumber eV = new TunableNumber("Elevator PID/V");
  private TunableNumber eS = new TunableNumber("Elevator PID/S");

  public Elevator() {
    elevator2.setControl(new Follower(Constants.IDs.elevator1 , false));    
    ampPosition.setDefault(ElevatorConstants.ampPosition);
    eTolerance.setDefault(ElevatorConstants.elevator1Tolerance);
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    mm_eVelocity.setDefault(ElevatorConstants.elevator1Velocity);
    mm_eAcceleration.setDefault(ElevatorConstants.elevator1Acceleration);
    mm_eJerk.setDefault(ElevatorConstants.elevator1Jerk);

    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = mm_eVelocity.get();
    mm.MotionMagicAcceleration = mm_eAcceleration.get();
    mm.MotionMagicJerk = mm_eJerk.get();

    eP.setDefault(ElevatorConstants.elevator1P);
    eI.setDefault(ElevatorConstants.elevator1I);
    eD.setDefault(ElevatorConstants.elevator1D);
    eV.setDefault(ElevatorConstants.elevator1V);
    eS.setDefault(ElevatorConstants.elevator1S);


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
      status = elevator1.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }

  public void periodic() {
    SmartDashboard.putNumber("Elevator/Reported Position: ", elevator1.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Reported Velocity: ", elevator1.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator/Reported Power:", elevator1.get());
    SmartDashboard.putNumber("Elevator/Reported Voltage:", elevator1.getMotorVoltage().getValueAsDouble());
  }

  public void ampExtendElevator() {
    elevator1.setControl(m_mmReq.withPosition(ElevatorConstants.ampPosition).withSlot(0));
  }

  public void stowElevator() {
    elevator1.setControl(m_mmReq.withPosition(0).withSlot(0));
  }

  public void zeroElevatorPosition() {
    elevator1.setPosition(0);
  }

  public boolean isElevatorSet() {
    return (m_mmReq.Position <= (elevator1.getPosition().getValueAsDouble()+eTolerance.get()) && m_mmReq.Position >= (elevator1.getPosition().getValueAsDouble()-eTolerance.get()));
  }

  public boolean isElevatorAmped() {
    return (ampPosition.get() <= (elevator1.getPosition().getValueAsDouble()+eTolerance.get()) && ampPosition.get() >= (elevator1.getPosition().getValueAsDouble()-eTolerance.get()));
  }

  public Command ampElevatorCommand() {
    return runOnce(
            () -> {
              elevator1.setControl(m_mmReq.withPosition(ampPosition.get()).withSlot(0));
            })
        .andThen(run(() -> {}).withTimeout(0.05))
        .withName("Elevator Lifted");
  }

  public Command fullElevatorCommand() {
    return runOnce(
            () -> {
              elevator1.setControl(m_mmReq.withPosition(1).withSlot(0));
            })
        .andThen(run(() -> {}).withTimeout(0.05))
        .withName("Elevator Lifted");
  }

  public Command stowElevatorCommand() {
    return runOnce(
            () -> {
              elevator1.setControl(m_mmReq.withPosition(0).withSlot(0));
            })
        .andThen(run(() -> {}).withTimeout(0.05))
        .withName("Elevator Stowed");
  }
}
