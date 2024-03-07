package frc.robot.subsystems.shooter;

public class ShooterConstants {
    public static double subwooferRPM = 2500;
    public static double subwooferAngle = 0;
    public static double podiumRPM = 3200;
    public static double podiumAngle = 0.14;

    public static double humanPlayerIntakeSpeed = -0.2;
    public static double groundIntakeAngle = 0.2;
    public static double TOFsensorRange = 335;

    public static double flywheelThreshold = 100;
    public static double angleThreshold = 0.01;

    public static double flywheelP = 6e-5;
    public static double flywheelI = 0;
    public static double flywheelD = 0;
    public static double flywheelIZ = 0;
    public static double flywheelFF = 0.000015;
    public static double flywheelMaxOutput = 1;
    public static double flywheelMinOutput = -1;
    public static double flywheelMaxRPM = 6000;

    public static double shooterVelocity = 30;
    public static double shooterAcceleration = 30;
    public static double shooterJerk = 100;
    public static double shooterP = 30;
    public static double shooterI = 20;
    public static double shooterD = 0.12;
    public static double shooterV = 0.12;
    public static double shooterS = 0.25;
}
