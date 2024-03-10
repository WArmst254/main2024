package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TunableNumber;

public class Elevator extends SubsystemBase {

  private final TalonFX elevator = new TalonFX(Constants.IDs.elevator);
  private final MotionMagicVoltage m_mmReq = new MotionMagicVoltage(0);

   private TunableNumber mm_eVelocity = new TunableNumber("Elevator/Velocity");
  private TunableNumber mm_eAcceleration = new TunableNumber("Elevator/Acceleration");
  private TunableNumber mm_eJerk = new TunableNumber("Elevator/Jerk");
  private TunableNumber eP = new TunableNumber("Elevator PID/P");
  private TunableNumber eI = new TunableNumber("Elevator PID/I");
  private TunableNumber eD = new TunableNumber("Elevator PID/D");
  private TunableNumber eV = new TunableNumber("Elevator PID/V");
  private TunableNumber eS = new TunableNumber("Elevator PID/S");

  public Elevator() {
    TalonFXConfiguration cfg = new TalonFXConfiguration();

    mm_eVelocity.setDefault(2000);
    mm_eAcceleration.setDefault(2000);
    mm_eJerk.setDefault(2000);

    MotionMagicConfigs mm = cfg.MotionMagic;
    mm.MotionMagicCruiseVelocity = mm_eVelocity.get();
    mm.MotionMagicAcceleration = mm_eAcceleration.get();
    mm.MotionMagicJerk = mm_eJerk.get();

    eP.setDefault(100);
    eI.setDefault(.2);
    eD.setDefault(.4);
    eV.setDefault(1.2);
    eS.setDefault(1);


    Slot0Configs slot0 = cfg.Slot0;
    slot0.kP = eP.get();
    slot0.kI = eI.get();
    slot0.kD = eD.get();
    slot0.kV = eV.get();
    slot0.kS = eS.get();

    FeedbackConfigs fdb = cfg.Feedback;
    fdb.SensorToMechanismRatio = 52;

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = elevator.getConfigurator().apply(cfg);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
  }

  public void periodic() {
    SmartDashboard.putNumber("Elevator Position: ", elevator.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Velocity: ", elevator.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Elevator Power:", elevator.get());
    SmartDashboard.putNumber("Elevator Voltage:", elevator.getMotorVoltage().getValueAsDouble());
  }

  public void fullExtendElevator() {
    elevator.setControl(m_mmReq.withPosition(-1).withSlot(0));
  }

  public void ampExtendElevator() {
    elevator.setControl(m_mmReq.withPosition(-0.65).withSlot(0));
  }

  public void stowElevator() {
    elevator.setControl(m_mmReq.withPosition(0).withSlot(0));
  }

  public void zeroElevatorPosition() {
    elevator.setPosition(0);
  }

  public boolean isElevatorSet() {
    return (m_mmReq.Position <= (elevator.getPosition().getValueAsDouble()+0.01) && m_mmReq.Position >= (elevator.getPosition().getValueAsDouble()-0.01));
  }

  public Command ampElevatorCommand() {
    return runOnce(
            () -> {
              elevator.setControl(m_mmReq.withPosition(-0.8).withSlot(0));
            })
        .andThen(run(() -> {}).withTimeout(0.5))
        .withName("Elevator Lifted");
  }

  public Command stowElevatorCommand() {
    return runOnce(
            () -> {
              elevator.setControl(m_mmReq.withPosition(0).withSlot(0));
            })
        .andThen(run(() -> {}).withTimeout(0.5))
        .withName("Elevator Stowed");
  }
}
