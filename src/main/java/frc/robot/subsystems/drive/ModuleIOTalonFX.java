package frc.robot.subsystems.drive;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;

import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.util.NRUnits;

public class ModuleIOTalonFX implements ModuleIO {

    private SwerveModule module;

    private SwerveModulePosition position;
    private SwerveModuleState state;
    
    private TalonFX moveMotor;
    private TalonFX turnMotor;
    private CANcoder encoder;

     public ModuleIOTalonFX(SwerveModuleConstants constants, String canbusName) {
        module = new SwerveModule(constants, canbusName);

        position = module.getPosition(true);
        state = module.getCurrentState();

        moveMotor = module.getDriveMotor();
        turnMotor = module.getSteerMotor();
        encoder = module.getCANcoder();

        module.getSteerMotor().getConfigurator().apply(new MotionMagicConfigs()
        .withMotionMagicCruiseVelocity(100.0 / Constants.Drive.kSteerGearRatio)
        .withMotionMagicAcceleration(100.0 / Constants.Drive.kSteerGearRatio / 0.2)
        .withMotionMagicExpo_kV(0.12 * Constants.Drive.kSteerGearRatio)
        .withMotionMagicExpo_kA(0.1));
        module.getDriveMotor().getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(true)
        .withStatorCurrentLimit(Constants.Drive.kSlipCurrentA)
        .withSupplyCurrentLimitEnable(true).
        withSupplyCurrentLimit(Constants.Drive.kSlipCurrentA));
    }    

    public void updateInputs(ModuleIOInputs inputs) {
        position = module.getPosition(true);
        state = module.getCurrentState();

        inputs.movePosition = position.distanceMeters;
        inputs.moveVelocity = Math.abs(module.getDriveMotor().getVelocity().getValueAsDouble() / Constants.Drive.kDriveGearRatio * (2*Math.PI) * Constants.Drive.WHEEL_RADIUS);
        inputs.moveVoltage = module.getDriveMotor().getMotorVoltage().getValueAsDouble();
        inputs.moveStatorCurrent = moveMotor.getStatorCurrent().getValue();
        inputs.moveSupplyCurrent = moveMotor.getSupplyCurrent().getValue();

        inputs.turnAbsolutePosition = NRUnits.logConstrainRad(encoder.getAbsolutePosition().getValue() * (2*Math.PI));
        inputs.turnRotorPosition = NRUnits.logConstrainRad(position.angle.getRadians());
        inputs.turnVelocity = turnMotor.getVelocity().getValue() * (2*Math.PI) / Constants.Drive.kSteerGearRatio;
        inputs.turnVoltage = turnMotor.getMotorVoltage().getValueAsDouble();
        inputs.turnStatorCurrent = turnMotor.getStatorCurrent().getValue();
        inputs.turnSupplyCurrent = turnMotor.getSupplyCurrent().getValue();
    }

    public void set(SwerveModuleState state) {
        if(state.speedMetersPerSecond == 0) {
            module.apply(
                new SwerveModuleState(0, module.getTargetState().angle),
                DriveRequestType.Velocity,
                SteerRequestType.MotionMagic
            );
        } else
        module.apply(state, DriveRequestType.Velocity, SteerRequestType.MotionMagic);
    }

    public void setBoltage(double voltage){
        module.getDriveMotor().setVoltage(voltage);
    }


    // public void disableCurrentLimit(){
    //     moveMotor.getConfigurator().apply(new CurrentLimitsConfigs()
    //     .withStatorCurrentLimitEnable(false)
    //     // .withStatorCurrentLimit(10)
    //     .withSupplyCurrentLimitEnable(false)
    //     .withSupplyCurrentThreshold(1000)
    //     );
    // }

    public SwerveModulePosition getPosition() {
        return position;
    }

    public SwerveModuleState getState() {
        return state;
    }

    public SwerveModule getModule() {
        return module;
    }

    public void enableMoveStatorLimit(boolean enable){
         module.getDriveMotor().getConfigurator().apply(new CurrentLimitsConfigs()
        .withStatorCurrentLimitEnable(enable)
        .withStatorCurrentLimit(Constants.Drive.kSlipCurrentA)
        .withSupplyCurrentLimitEnable(true).
        withSupplyCurrentLimit(Constants.Drive.kSlipCurrentA));
    }
}