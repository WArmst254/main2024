// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase {
    CANdle candle = new CANdle(20);

    public void configLEDs() {
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = true; //will be diff between hold and press
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.5;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        candle.configAllSettings(configAll, 100);
    }

  public void candleOn(int red, int green, int blue){
    candle.setLEDs(red, green, blue);
  }
  
  public void candleChunkOn(int red, int green, int blue, int white, int startindex, int count) {
    candle.setLEDs(red, green, blue, white, startindex, count);
  }

  public void rainbowAnimation(double brightness, double speed, int numLeds){
    FireAnimation rainbowAnim = new FireAnimation(brightness, speed, numLeds+8, 0.8, 0.4);
    candle.animate(rainbowAnim);   
  }

  public void larsonAnimation(int numLeds){
  LarsonAnimation larsonAnimation = new LarsonAnimation(250, 40, 146, 0, 0.6, numLeds, BounceMode.Front, 7);
  candle.animate(larsonAnimation);
  }

}