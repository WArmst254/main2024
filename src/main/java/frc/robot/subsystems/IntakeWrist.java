// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeWrist extends SubsystemBase {
  public final TalonFX wristMotor = new TalonFX(17);
  public final PIDController pid = new PIDController(0.4, .01, 0);
  private double setPoint = 0;

  // Constants
  private static final int BOTTOM_ENCODER_VALUE = 0;
  private static final int TOP_ENCODER_VALUE = -15;
  private static final double MAX_POWER = .6;

  public void intakeWrist() {

    this.pid.setTolerance(0.1, 0.07 / 20);
    this.wristMotor.setNeutralMode(NeutralModeValue.Brake);
    ;
    this.wristMotor.setInverted(false);
    this.setSetpoint(this.getEncoderValue());
  }

  public void IntakeWrist_On(Double power) {
    wristMotor.set(power);
  }

  private double getEncoderValue() {
    return (this.wristMotor.getPosition().getValueAsDouble() - BOTTOM_ENCODER_VALUE)
        / (TOP_ENCODER_VALUE - BOTTOM_ENCODER_VALUE);
  }

  private double getPidPower() {
    double power = this.pid.calculate(this.getEncoderValue());
    return Math.min(MAX_POWER, Math.max(-MAX_POWER, power));
  }

  /**
   * Set the current setpoint for where the IntakeWrist should move to
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
    SmartDashboard.putNumber("IntakeWrist Encoders", wristMotor.getPosition().getValueAsDouble());
    SmartDashboard.putNumber("Current Setpoint", this.setPoint);
    SmartDashboard.putNumber("Motor Power", this.wristMotor.get());
    SmartDashboard.putNumber("Transform Encoder Value", getEncoderValue());
    SmartDashboard.putNumber("expected motor power", getPidPower());
    wristMotor.set(-getPidPower());
  }

  public void zero() {
    wristMotor.setPosition(0);
  }

  public void brakedis() {
    wristMotor.setNeutralMode(NeutralModeValue.Coast);
  }
  // IntakeWrist drives to PID
  public void driveTowardsPid() {
    // double pidPower = this.getPidPower();

  }
}
