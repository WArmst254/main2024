package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.Setpoints;
import frc.robot.Constants;
import frc.robot.util.ErrorCheckUtil;
import frc.robot.util.ErrorCheckUtil.CommonErrorNames;
import frc.robot.util.TalonFXFactory;

public class Elevator extends SubsystemBase {
    
  private TalonFX elevatorLeaderTalon = configureElevatorTalon(
      TalonFXFactory.createTalon(ElevatorConstants.elevatorLeaderTalonID,
          ElevatorConstants.elevatorTalonCANBus, ElevatorConstants.kElevatorConfiguration));

  private TalonFX elevatorFollowerTalon = configureElevatorTalon(
      TalonFXFactory.createTalon(ElevatorConstants.elevatorFollowerTalonID,
          ElevatorConstants.elevatorTalonCANBus, ElevatorConstants.kElevatorConfiguration));

  public Elevator() {
    elevatorFollowerTalon.setControl(ElevatorConstants.followerControl);
  }

  /**
   * Move Elevator to position
   * 
   * @param height in meters (0 to max height)
   */
  public void setHeight(double height) {

    elevatorLeaderTalon.setControl(
          ElevatorConstants.elevatorPositionControl.withPosition(ElevatorConstants.elevatorMetersToRotations(height)));
    elevatorFollowerTalon.setControl(ElevatorConstants.followerControl);
  }

  /**
   * Move elevator to home position (0)
   */
  public void stow() {
    setHeight(Setpoints.ElevatorStowHeight);
  }

  public void holdPosition() {

    elevatorLeaderTalon.setControl(new VoltageOut(ElevatorConstants.kElevatorConfiguration.Slot0.kG));
    elevatorFollowerTalon.setControl(ElevatorConstants.followerControl);
  }

  /**
   * Set all outputs to 0
   */
  public void stop() {

    elevatorLeaderTalon.setControl(new DutyCycleOut(0));
    elevatorFollowerTalon.setControl(new DutyCycleOut(0));
  }

  /**
   * Run elevator motors at a voltage
   * 
   * @param volts
   */
  public void setElevatorVoltage(double volts) {

    elevatorLeaderTalon.setControl(new VoltageOut(volts));
    elevatorFollowerTalon.setControl(ElevatorConstants.followerControl);
  }

  public void resetEncoderPosition(double height) {

    elevatorLeaderTalon.setPosition(ElevatorConstants.elevatorMetersToRotations(height));
    elevatorFollowerTalon.setPosition(ElevatorConstants.elevatorMetersToRotations(height));
  }

  public double getSetpointError() {
    return ElevatorConstants.elevatorRotationsToMeters(elevatorLeaderTalon.getClosedLoopError().getValue());
  }

  public double getLeaderPosition() {
    return ElevatorConstants.elevatorRotationsToMeters(elevatorLeaderTalon.getPosition().getValue());
  }

  public double getFollowerPosition() {
    return ElevatorConstants.elevatorRotationsToMeters(elevatorFollowerTalon.getPosition().getValue());
  }

  public boolean isAtSetpoint() {
    return Math.abs(getSetpointError()) < ElevatorConstants.heightErrorTolerance;
  }

  @Override
  public void periodic() {
  }

    private TalonFX configureElevatorTalon(TalonFX motor) {

    ErrorCheckUtil.checkError(
        motor.getPosition().setUpdateFrequency(ElevatorConstants.kElevatorMidUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
    ErrorCheckUtil.checkError(
        motor.getClosedLoopError().setUpdateFrequency(ElevatorConstants.kElevatorMidUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
    ErrorCheckUtil.checkError(
        motor.getStatorCurrent().setUpdateFrequency(ElevatorConstants.kElevatorMidUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
    ErrorCheckUtil.checkError(
        motor.getReverseLimit().setUpdateFrequency(ElevatorConstants.kElevatorFastUpdateFrequency,
            Constants.kConfigTimeoutSeconds),
        CommonErrorNames.UpdateFrequency(motor.getDeviceID()));
    return motor;
  }
}