// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.led;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FieldUtil;

public class LED extends SubsystemBase {
    private static LED led = null;
    CANdle candle = new CANdle(20);

    private int[][] ledZones = new int[4][2];

    public enum LEDState {
      OFF,
      DISABLED,
      DISABLED_NO_CONTROLLERS,
      E_STOPPED,
      PREMATCH,
      AUTON,

      AMP_ABSENT,
      SHOOTER_ABSENT,
      AMP_INTAKING,
      SHOOTER_INTAKING,
      AMP_INTAKE_SUCCESSFUL,
      SHOOTER_INTAKE_SUCCESSFUL,
      READY_TO_SHOOT,
      READY_TO_AMP,
      AMPING,
      SHOOTING,
      FIRED,
      HP_INTAKE;
  }

  private LEDState currentState = LEDState.DISABLED;

  public LED() {
      changeLedState(LEDState.OFF);
      CANdleConfiguration configAll = new CANdleConfiguration();
      configAll.statusLedOffWhenActive = true;
      configAll.disableWhenLOS = false;
      configAll.stripType = LEDStripType.GRB;
      configAll.brightnessScalar = 0.99;
      configAll.vBatOutputMode = VBatOutputMode.Modulated;

      candle.configAllSettings(configAll, 100);
      candle.configLOSBehavior(true);

      ledZones[0][0] = 8;
      ledZones[0][1] = 30;
  
      ledZones[1][0] = 39;
      ledZones[1][1] = 23;
  
      ledZones[2][0] = 60;
      ledZones[2][1] = 27;
  
      ledZones[3][0] = 90;
      ledZones[3][1] = 26;
  }

   public void changeLedState(LEDState state) {

        for (int i = 0; i < 10; ++i) {
          candle.clearAnimation(i);
        }
        candle.setLEDs(0, 0, 0, 0, 8, 102);

        switch (state) {
            default:
            break;

            case DISABLED:
            candle.animate(new ColorFlowAnimation(255, 255, 255, 0, 0.5, ledZones[0][1] + ledZones[1][1], Direction.Backward, ledZones[0][0]), 1);
            candle.animate(new ColorFlowAnimation(255, 255, 255, 0, 0.5, ledZones[2][1] + ledZones[3][1] - 1, Direction.Forward, ledZones[2][0]), 2);      
            break;

            case DISABLED_NO_CONTROLLERS:
            candle.animate(new ColorFlowAnimation(255, 255, 255, 0, 0.5, ledZones[0][1] + ledZones[1][1], Direction.Backward, ledZones[0][0]), 1);
            candle.animate(new ColorFlowAnimation(255, 255, 255, 0, 0.5, ledZones[2][1] + ledZones[3][1] - 1, Direction.Forward, ledZones[2][0]), 2);      
            break;

            case AUTON:
            candle.animate(
              new FireAnimation(1, 0.7, ledZones[0][1], 0.7, 0.25, false, ledZones[0][0]), 1);
      candle.animate(
              new FireAnimation(1, 0.7, ledZones[1][1], 0.7, 0.25, true, ledZones[1][0]), 2);
      candle.animate(
              new FireAnimation(1, 0.7, ledZones[2][1], 0.7, 0.25, false, ledZones[2][0]), 3);
      candle.animate(
              new FireAnimation(1, 0.7, ledZones[3][1], 0.7, 0.25, true, ledZones[3][0]), 4);   
              currentState = LEDState.AUTON;
            break;

            case PREMATCH:
                currentState = LEDState.PREMATCH;
                break;

            case HP_INTAKE:
              candle.animate(new StrobeAnimation(255,77,0,0,.5, 112));
              currentState = LEDState.HP_INTAKE;

            case OFF:
              currentState = LEDState.OFF;
            break;

            case SHOOTER_INTAKING:
            candle.animate(new StrobeAnimation(255,128,0,0,.5, 112));
            currentState = LEDState.SHOOTER_INTAKING;

            case SHOOTER_ABSENT: 
            candle.animate(new SingleFadeAnimation(255, 128, 0, 0, 0.6, 112, 8));
            currentState = LEDState.SHOOTER_ABSENT;
            break;

            case SHOOTER_INTAKE_SUCCESSFUL: 
            candle.setLEDs(0,255,0, 0, 8, 112);

            case SHOOTING:
            candle.animate(new LarsonAnimation(255, 128, 0, 0, 0.5, ledZones[0][1] + ledZones[1][1], BounceMode.Center, ledZones[0][0], 0), 1);
            candle.animate(new LarsonAnimation(255, 128, 0, 0, 0.5, ledZones[2][1] + ledZones[3][1] - 1, BounceMode.Center, ledZones[2][0], 0), 2);      

            case AMP_INTAKING:
            candle.animate(new StrobeAnimation(255,0,128,0,.5, 112));
            currentState = LEDState.AMP_INTAKING;

            case AMP_ABSENT: 
            candle.animate(new SingleFadeAnimation(255, 0, 128, 0, 0.6, 112, 8));
            currentState = LEDState.AMP_ABSENT;
            break;

            case AMP_INTAKE_SUCCESSFUL: 
            candle.setLEDs(0,255,0, 0, 8, 112);
            currentState = LEDState.AMP_INTAKE_SUCCESSFUL;
            break;

            case AMPING:
            candle.animate(new LarsonAnimation(255, 0, 128, 0, 0.5, ledZones[0][1] + ledZones[1][1], BounceMode.Center, ledZones[0][0], 0), 1);
            candle.animate(new LarsonAnimation(255, 0, 128, 0, 0.5, ledZones[2][1] + ledZones[3][1] - 1, BounceMode.Center, ledZones[2][0], 0), 2);      

        }

            
    }

    public void periodic() {

      if(DriverStation.isEStopped()) {
        candle.animate(new StrobeAnimation(255,0,0,0,.5, 112));
      }

       if (currentState == LEDState.PREMATCH) {

            if (NetworkTableInstance.getDefault().isConnected()) {
              
                if (!FieldUtil.isAllianceBlue()) {
                    // Red team
                    candle.animate(new ColorFlowAnimation(255, 0, 0, 0, 0.5, ledZones[0][1] + ledZones[1][1], Direction.Backward, ledZones[0][0]), 1);
                    candle.animate(new ColorFlowAnimation(255, 0, 0, 0, 0.5, ledZones[2][1] + ledZones[3][1] - 1, Direction.Forward, ledZones[2][0]), 2);      

                } else {
                    // Blue Team
                    candle.animate(new ColorFlowAnimation(0, 0, 255, 0, 0.5, ledZones[0][1] + ledZones[1][1], Direction.Backward, ledZones[0][0]), 1);
                    candle.animate(new ColorFlowAnimation(0, 0, 255, 0, 0.5, ledZones[2][1] + ledZones[3][1] - 1, Direction.Forward, ledZones[2][0]), 2);      

                }
            } else {

              candle.animate(new ColorFlowAnimation(255, 255, 255, 0, 0.5, ledZones[0][1] + ledZones[1][1], Direction.Backward, ledZones[0][0]), 1);
              candle.animate(new ColorFlowAnimation(255, 255, 255, 0, 0.5, ledZones[2][1] + ledZones[3][1] - 1, Direction.Forward, ledZones[2][0]), 2);      
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