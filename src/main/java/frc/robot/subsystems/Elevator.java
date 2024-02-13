// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  public final TalonFX eleMotor = new TalonFX(20);
  public final PIDController pid = new PIDController(0.4, .01, 0);
  private double setPoint = 0;

  // Constants
  private static final int BOTTOM_ENCODER_VALUE = 0;
  private static final int TOP_ENCODER_VALUE = -19;
  private static final double MAX_POWER = .7;

  public void elevator() {

    this.pid.setTolerance(0.1, 0.07 / 20);
    this.eleMotor.setNeutralMode(NeutralModeValue.Brake);
    ;
    this.eleMotor.setInverted(false);
    this.setSetpoint(this.getEncoderValue());
  }

  public void Elevator_On(Double power) {
    eleMotor.set(power);
  }

  private double getEncoderValue() {
    return (this.eleMotor.getPosition().getValueAsDouble() - BOTTOM_ENCODER_VALUE)
        / (TOP_ENCODER_VALUE - BOTTOM_ENCODER_VALUE);
  }

  private double getPidPower() {
    double power = this.pid.calculate(this.getEncoderValue());
    return Math.min(MAX_POWER, Math.max(-MAX_POWER, power));
  }

  /**
   * Set the current setpoint for where the elevator should move to
   *
   * @param setpoint Value between 0 and 1
   */
  public void setSetpoint(double setpoint) {
    this.setPoint = setpoint;
    this.pid.setSetpoint(setpoint);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // double Vdegrees = Vcoder.getPosition();
    SmartDashboard.putNumber("Elevator Encoders", eleMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Current Setpoint", this.setPoint);
    SmartDashboard.putNumber("Motor Power", this.eleMotor.get());
    SmartDashboard.putNumber("Transform Encoder Value", getEncoderValue());
    SmartDashboard.putNumber("expected motor power", getPidPower());
    eleMotor.set(-getPidPower());
  }

  public void zero() {
    eleMotor.setPosition(0);
  }

  public void brakedis() {
    eleMotor.setNeutralMode(NeutralModeValue.Coast);
  }
  // Elevator drives to PID
  public void driveTowardsPid() {
    // double pidPower = this.getPidPower();

  }
}
