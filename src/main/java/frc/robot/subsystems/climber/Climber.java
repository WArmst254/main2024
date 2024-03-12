package frc.robot.subsystems.climber;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final TalonFX climber1 = new TalonFX(30);
    private final TalonFX climber2 = new TalonFX(31);

    public void climber(){

    }
    
    public void climbUp(){
        climber1.set(.1);
        climber2.set(.1);
    }

    public void climbDown(){
        climber1.set(-.1);
        climber2.set(-.1);
    }

    public void climbOff(){
        climber1.set(0);
        climber2.set(0);
        climber1.setNeutralMode(NeutralModeValue.Brake);
        climber2.setNeutralMode(NeutralModeValue.Brake);
    }
}
