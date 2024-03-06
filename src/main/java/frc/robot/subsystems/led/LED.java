// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;
import java.util.Optional;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LED extends SubsystemBase {
    private static LED led = null;
    CANdle candle = new CANdle(Constants.IDs.led);
    private Optional<Alliance> alliance = Optional.empty();

    public enum LEDState {
      IDLE, 
      DISABLED,
      PREMATCH,
      AUTON,
      OFF,

      SPEAKER,
      AMP,


      HP_AMPLIFY,
      HP_COOPERTITION,

      INTAKING,
      SUBWOOFER_SHOOTING,
      SHOT_FIRED,

      HP_INTAKE_DETECTED,
      GROUND_INTAKE_DETECTED,
  }

  private LEDState currentState = LEDState.OFF;

  public LED() {
      changeLedState(LEDState.OFF);
      CANdleConfiguration configAll = new CANdleConfiguration();
      configAll.statusLedOffWhenActive = true;
      configAll.disableWhenLOS = false;
      configAll.stripType = LEDStripType.GRB;
      configAll.brightnessScalar = 0.1;
      configAll.vBatOutputMode = VBatOutputMode.Modulated;

      candle.configAllSettings(configAll, 100);
      candle.configLOSBehavior(true);
  }

   public void changeLedState(LEDState state) {

        for (int i = 0; i < 10; ++i) {
          candle.clearAnimation(i);
        }
        candle.setLEDs(0, 0, 0, 0, 8, 48);

        switch (state) {
            case IDLE:
              candle.animate(new SingleFadeAnimation(255,255,255,255,0.9,48,8));
              currentState = LEDState.IDLE;
            break;

            case DISABLED:
              candle.animate(new RainbowAnimation(0.7, 0.9, 48, false,8));
              currentState = LEDState.DISABLED;
            break;

            case PREMATCH:
              currentState = LEDState.PREMATCH;
            break;

            case AUTON:
              candle.animate(new FireAnimation(0.7, 0.8, 48, .7, .5,false,8));
              currentState = LEDState.AUTON;
            break;

            case OFF:
              currentState = LEDState.OFF;
            break;

            default:
            break;

            case SPEAKER:
              candle.setLEDs(255,0,255, 0, 8, 48);
              currentState = LEDState.SPEAKER;
            break;
            case INTAKING:
              candle.setLEDs(255,40,255, 0, 8, 48);
              currentState = LEDState.INTAKING;
            break;

            case AMP:
              candle.setLEDs(255,160,0, 0, 8, 48);
              currentState = LEDState.AMP;
            break;
            case SUBWOOFER_SHOOTING:
            candle.animate(new LarsonAnimation(255, 0, 178, 255, 1, 48, BounceMode.Center, 8));
            break;
            case SHOT_FIRED:
            candle.animate(new FireAnimation(0.7, 0.8, 48, .7, .5,false,8));
            break;

            case HP_AMPLIFY:
              candle.animate(new StrobeAnimation(0,0,255,0,.5, 48, 8));
              currentState = LEDState.HP_AMPLIFY;
            break;

            case HP_COOPERTITION:
              candle.animate(new StrobeAnimation(255,255,0,0,.5, 48, 8));
              currentState = LEDState.HP_COOPERTITION;
            break;

            case GROUND_INTAKE_DETECTED:
              candle.setLEDs(0,255,0,0,8, 48);
              currentState = LEDState.GROUND_INTAKE_DETECTED;
            break;

            case HP_INTAKE_DETECTED:
              candle.setLEDs(0,255,0,0,28,48);
              currentState = LEDState.HP_INTAKE_DETECTED;
            break;

        }
    }

    public void periodic() {
      alliance = DriverStation.getAlliance();
      if(!DriverStation.isJoystickConnected(0) || !DriverStation.isJoystickConnected(1)) {
        candle.animate(new TwinkleAnimation(255, 0, 178, 255, 0.9, 48, TwinklePercent.Percent64,8));
      }

      if(DriverStation.isEStopped()) {
        candle.animate(new StrobeAnimation(255,0,0,0,.5, 48, 8));
      }
      if (currentState == LEDState.PREMATCH) {

          if (NetworkTableInstance.getDefault().isConnected()) {
            
              if (alliance.get() == Alliance.Red){
                  // Red team
                  candle.animate(new SingleFadeAnimation(255, 0, 0, 0, 0.2, 48, 8), 1);

              } else {
                  // Blue Team
                  candle.animate(new SingleFadeAnimation(0, 0, 255, 0, 0.2, 48, 8), 1);
              }
          } else {
              candle.animate(new SingleFadeAnimation(255, 0, 255, 0, 0.2, 48, 8), 1);
          }
        }
      }

    public static LED getInstance() {
      // To ensure only one instance is created
      if (led == null) {
           led = new LED();
      }
      return led;
  }
    
}