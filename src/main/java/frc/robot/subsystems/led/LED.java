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
      CONTROLLERS_DISCONNECTED,
      PREMATCH,
      AUTON,
      OFF,

      SPEAKER,
      AMP,

      INTAKING_SPEAKER,
      INTAKE_SUCCESS_SPEAKER,

      INTAKING_AMP,
      INTAKE_SUCCESS_AMP,

      MANUAL_SHOOTING,
      VISION_SHOOTING,

      AMP_SCORING,

      SHOT_FIRED,

      NOTE_IN_FEED,
      NO_VISION,

      REQUEST_HP_INTAKE,
      HP_AMPLIFY,
      HP_COOPERTITION
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
        candle.setLEDs(0, 0, 0, 0, 0, 48);

        switch (state) {
            default:
            break;

            case IDLE:
              candle.animate(new SingleFadeAnimation(255,255,255,255,0.9,48,0));
              currentState = LEDState.IDLE;
            break;

            case DISABLED:
              candle.animate(new RainbowAnimation(0.7, 0.9, 300, false,0));
              currentState = LEDState.DISABLED;
            break;

            case PREMATCH:
              currentState = LEDState.PREMATCH;
            break;

            case CONTROLLERS_DISCONNECTED:
              candle.animate(new TwinkleAnimation(255, 0, 178, 255, 1, 400, TwinklePercent.Percent64,0));
              currentState = LEDState.CONTROLLERS_DISCONNECTED;
            break;

            case AUTON:
              candle.animate(new FireAnimation(0.7, 0.8, 48, .7, .5,false, 0));
              currentState = LEDState.AUTON;
            break;

            case OFF:
              currentState = LEDState.OFF;
            break;

            case SPEAKER:
              candle.setLEDs(128,0,255, 0, 0, 48);
              currentState = LEDState.SPEAKER;
            break;

            case AMP:
              candle.setLEDs(255,0,0, 0, 0, 48);
              currentState = LEDState.AMP;
            break;

            case INTAKING_SPEAKER:
            candle.animate(new SingleFadeAnimation(128, 0, 255, 0, 0.9, 48), 1);
              currentState = LEDState.INTAKING_SPEAKER;
            break;

            case INTAKING_AMP:
              candle.animate(new SingleFadeAnimation(255, 0, 0, 0, 0.9, 48), 1);
              currentState = LEDState.INTAKING_AMP;
            break;

            case MANUAL_SHOOTING:
              candle.animate(new LarsonAnimation(128, 0, 255, 0, 1, 48, BounceMode.Center, 7));
              currentState = LEDState.MANUAL_SHOOTING;
            break;

            case VISION_SHOOTING:
              candle.animate(new LarsonAnimation(0, 255, 0, 0, 1, 48, BounceMode.Center, 7));
              currentState = LEDState.VISION_SHOOTING;
            break;

            case NOTE_IN_FEED: 
              candle.animate(new SingleFadeAnimation(0, 0, 0, 255, 1, 48, 8), 1);
              currentState = LEDState.NOTE_IN_FEED;
            break;

            case AMP_SCORING:
              candle.animate(new LarsonAnimation(255, 0, 0, 0, 1, 48, BounceMode.Center, 7));
              currentState = LEDState.AMP_SCORING;
            break;

            case NO_VISION:
              candle.animate(new StrobeAnimation(255,0,0,0,.3, 48));
              currentState = LEDState.NO_VISION;
            break;

            case SHOT_FIRED:
              candle.animate(new FireAnimation(0.9, 0.8, 48, .7, .5,false, 0));
              currentState = LEDState.SHOT_FIRED;
            break;

            case HP_AMPLIFY:
              candle.animate(new StrobeAnimation(0,0,255,0,.5, 48));
              currentState = LEDState.HP_AMPLIFY;
            break;

            case HP_COOPERTITION:
              candle.animate(new StrobeAnimation(255,255,0,0,.5, 48));
              currentState = LEDState.HP_COOPERTITION;
            break;

            case REQUEST_HP_INTAKE:
              candle.animate(new StrobeAnimation(255,128,0,0,.5, 48));
              currentState = LEDState.REQUEST_HP_INTAKE;
            break;


            case INTAKE_SUCCESS_SPEAKER:
              candle.setLEDs(128,0,255,0,0, 24);
              candle.setLEDs(0,255,0,0,24, 20);
              currentState = LEDState.INTAKE_SUCCESS_SPEAKER;
            break;

            case INTAKE_SUCCESS_AMP:
              candle.setLEDs(255,0,0,0,0, 24);
              candle.setLEDs(0,255,0,0,24, 20);
              currentState = LEDState.INTAKE_SUCCESS_SPEAKER;
            break;

        }
    }

    public void periodic() {
      alliance = DriverStation.getAlliance();

      if(DriverStation.isEStopped()) {
        candle.animate(new StrobeAnimation(255,0,0,0,.5, 48));
      }

      if (currentState == LEDState.PREMATCH) {
              if (alliance.get() == Alliance.Red){
                  // Red team
                  candle.animate(new SingleFadeAnimation(255, 0, 0, 0, 0.2, 48), 1);
              } else {
                  // Blue Team
                  candle.animate(new SingleFadeAnimation(0, 0, 255, 0, 0.2, 48), 1);
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