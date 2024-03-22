package frc.robot.subsystems.amp;

import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.TunableNumber;

public class Amp extends SubsystemBase {
    private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
    private final NeutralOut neutralOut = new NeutralOut();
   private TunableNumber ampSensorThreshold = new TunableNumber("Sensors/Amp Sensor Threshold");

  private final TalonFX ampMotor;
  public final TimeOfFlight amp_sensor;


  public Amp() {
    ampMotor = new TalonFX(Constants.IDs.amp);
    /*Sensor Shtuff */
    amp_sensor = new TimeOfFlight(Constants.IDs.ampsensor);
    amp_sensor.setRangingMode(TimeOfFlight.RangingMode.Short, 0.001);
    amp_sensor.setRangeOfInterest(6, 6, 14, 14);
    ampSensorThreshold.setDefault(AmpConstants.sensorThreshold);

  }

  public void ampOuttakeOn() {
    ampMotor.setControl(voltageOut.withOutput(10));
  }

  public void ampIntakeOn() {
    ampMotor.setControl(voltageOut.withOutput(-10));
  }

  public void disableAmp() {
    ampMotor.setControl(neutralOut);
  }

  public boolean ampSensorOut() {
    return (amp_sensor.getRange() < ampSensorThreshold.get());
  }

  public double ampSensor() {
    return amp_sensor.getRange();
  }

  public void periodic() {
    SmartDashboard.putNumber("Amp/Reported Power: ", ampMotor.get());
  }
}
