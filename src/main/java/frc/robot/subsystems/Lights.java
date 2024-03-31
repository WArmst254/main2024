// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.FieldUtil;

public class Lights extends SubsystemBase {

  private static Lights CANDLE = null;

  private CANdle candle = new CANdle(20, "rio");

  private static final int TOTAL_STRIP_LENGTH = 247; 
  private static final int OFFSET_LENGTH = 8; // LEDs to skip at the front of the strip, 8 means no lights on the CANDLE are on.


  // The first dimension is for individual zones.
  // Within the second dimension, the first number is the start idx for the zone,
  // and the second is the # of leds in that zone
  private int[][] ledZones = new int[4][2];

  public enum IntakeState {
    AMP, 
    SPEAKER, 
    NONE
  }

  private IntakeState currentGamePiece = IntakeState.NONE;

  public enum LEDState {
      Idle,
      Disabled,
      No_Controller,
      Battery_Low,
      PreMatch,
      Off,

      Fire,
      Intake,
      Outtake,
      Climb;
  }

  private LEDState currentState = LEDState.Off;


  public Lights() {
      changeLedState(LEDState.Off);
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

    public void changeCurrentGamePiece(IntakeState piece) {

        if(currentGamePiece != piece) {
            currentGamePiece = piece;
            if(currentState == LEDState.Intake || currentState == LEDState.Outtake){
                changeLedState(currentState);
            }
        }
    }

    public void changeLedState(LEDState state) {

        for (int i = 0; i < 10; ++i) {
            candle.clearAnimation(i);
        }
        candle.setLEDs(0, 0, 0, 0, OFFSET_LENGTH, 128);
        switch (state) {
            case Idle:
            // candle.animate(new LarsonAnimation(255, 255, 255, 0, 1, TOTAL_STRIP_LENGTH, BounceMode.Front, 7,
            //         OFFSET_LENGTH));

            candle.animate(new ColorFlowAnimation(255, 255, 255, 0, 0.5, ledZones[0][1] + ledZones[1][1], Direction.Backward, OFFSET_LENGTH + ledZones[0][0]), 1);
            candle.animate(new ColorFlowAnimation(255, 255, 255, 0, 0.5, ledZones[2][1] + ledZones[3][1] - 1, Direction.Forward, OFFSET_LENGTH + ledZones[2][0]), 2);      
        

            currentState = LEDState.Idle;
            
            break;
            case Disabled:
            candle.animate(new TwinkleAnimation(255, 255, 255, 0, 0.4,TOTAL_STRIP_LENGTH, TwinklePercent.Percent30,
                    OFFSET_LENGTH + ledZones[0][0]), 1);

                currentState = LEDState.Disabled;
                break;

            case PreMatch:
                currentState = LEDState.PreMatch;
                break;

            case Off:

                currentState = LEDState.Off;
                break;

            default:
                break;

            // *************** Robot-Specific Animations ***************** //
            case Fire: // TODO Tune the reverse directions
                candle.animate(
                        new FireAnimation(1, 0.7, ledZones[0][1], 0.8, 0.25, false, OFFSET_LENGTH + ledZones[0][0]), 1);
                candle.animate(
                        new FireAnimation(1, 0.7, ledZones[1][1], 0.8, 0.25, true, OFFSET_LENGTH + ledZones[1][0]), 2);
                candle.animate(
                        new FireAnimation(1, 0.7, ledZones[2][1], 0.8, 0.25, false, OFFSET_LENGTH + ledZones[2][0]), 5);
                candle.animate(
                        new FireAnimation(1, 0.7, ledZones[3][1], 0.8, 0.25, true, OFFSET_LENGTH + ledZones[3][0]), 4);
                currentState = LEDState.Fire;
                break;

            case Intake:
          
                intakeState(true);

                currentState = LEDState.Intake;

                break;
            case Outtake:

            intakeState(false);

                currentState = LEDState.Outtake;
                break;
        }
    }


    private void intakeState(boolean isForward) {

        switch (currentGamePiece) {
            case AMP:
            candle.animate(new ColorFlowAnimation(255, 0, 255, 0, 0.9, ledZones[0][1] + ledZones[1][1], isForward ? Direction.Backward : Direction.Forward, OFFSET_LENGTH + ledZones[0][0]), 1);
            candle.animate(new ColorFlowAnimation(255, 0, 255, 0, 0.9, ledZones[2][1] + ledZones[3][1] - 1, !isForward ? Direction.Backward : Direction.Forward, OFFSET_LENGTH + ledZones[2][0]), 2);      
        
                break;

            case SPEAKER:
            candle.animate(new ColorFlowAnimation(168, 255, 0, 0, 0.9, ledZones[0][1] + ledZones[1][1], isForward ? Direction.Backward : Direction.Forward, OFFSET_LENGTH + ledZones[0][0]), 1);
            candle.animate(new ColorFlowAnimation(168, 255, 0, 0, 0.9, ledZones[2][1] + ledZones[3][1] - 1, !isForward ? Direction.Backward : Direction.Forward, OFFSET_LENGTH + ledZones[2][0]), 2);      
        
                break;

            case NONE:

            candle.animate(new ColorFlowAnimation(255, 0, 0, 0, 0.9, ledZones[0][1] + ledZones[1][1], isForward ? Direction.Backward : Direction.Forward, OFFSET_LENGTH + ledZones[0][0]), 1);
            candle.animate(new ColorFlowAnimation(255, 0, 0, 0, 0.9, ledZones[2][1] + ledZones[3][1] - 1, !isForward ? Direction.Backward : Direction.Forward, OFFSET_LENGTH + ledZones[2][0]), 2);      

                break;
        }

        
    }
    

    public void periodic() {

        if (currentState == LEDState.PreMatch) {

            if (NetworkTableInstance.getDefault().isConnected()) {
              

                if (!FieldUtil.isAllianceBlue()) {
                    // Red team

                    candle.animate(new SingleFadeAnimation(255, 0, 0, 0, 0.1, TOTAL_STRIP_LENGTH, OFFSET_LENGTH), 1);

                } else {
                    // Blue Team
                    candle.animate(new SingleFadeAnimation(0, 0, 255, 0, 0.1, TOTAL_STRIP_LENGTH, OFFSET_LENGTH), 1);

                }
            } else {

                candle.animate(new SingleFadeAnimation(255, 0, 255, 0, 0.1, TOTAL_STRIP_LENGTH, OFFSET_LENGTH), 1);
            }
        }

    }

    public double getVbat() {
        return candle.getBusVoltage();
    }

    public double get5V() {
        return candle.get5VRailVoltage();
    }

    public double getCurrent() {
        return candle.getCurrent();
    }

    public double getTemperature() {
        return candle.getTemperature();
    }

    public void configBrightness(double percent) {
        candle.configBrightnessScalar(percent, 0);
    }

    public void configLos(boolean disableWhenLos) {
        candle.configLOSBehavior(disableWhenLos, 0);
    }

    public void configLedType(LEDStripType type) {
        candle.configLEDType(type, 0);
    }

    public void configStatusLedBehavior(boolean offWhenActive) {
        candle.configStatusLedState(offWhenActive, 0);
    }

    public void setAllToColor(int r, int g, int b) {

        candle.setLEDs(r, g, b, 255, 0, TOTAL_STRIP_LENGTH);

    }
    public void setCandleLeftToColor(Color color) {
        candle.setLEDs((int)color.red, (int)color.green, (int)color.blue, 255, 0, 4);

    }
    public void setCandleRightToColor(Color color) {
        candle.setLEDs((int)color.red, (int)color.green, (int)color.blue, 255, 4, 4);

    }

    /**
     * @return The Single instance of Singleton LimeLight
     */
    public static Lights getInstance() {
        // To ensure only one instance is created
        if (CANDLE == null) {
            CANDLE = new Lights();
        }
        return CANDLE;
    }
    
}