package frc.robot.commands;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.util.JoystickValues;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.drive.DriveSubsystem.DriveState;
import frc.robot.subsystems.joystick.JoystickSubsystem;

public class DriveCommand extends Command{
    
    private DriveSubsystem drive;
    private JoystickSubsystem joysticks;

    private JoystickValues leftJoystickValues;
    private JoystickValues rightJoystickValues;

    private ChassisSpeeds chassisSpeeds;


    private ProfiledPIDController angleController = new ProfiledPIDController(
        0, 0, 0, 
        new Constraints(Constants.Drive.MAX_ROTATION_VELOCITY, Constants.Drive.MAX_ROTATION_ACCELERATION)
        );

    public DriveCommand(DriveSubsystem drive, JoystickSubsystem joysticks) {
        this.drive = drive;
        this.joysticks = joysticks;

        addRequirements(drive);
        chassisSpeeds = new ChassisSpeeds(0, 0, 0);
        leftJoystickValues = new JoystickValues(0, 0);
        rightJoystickValues = new JoystickValues(0, 0);

    }

    @Override
    public void initialize() {
        angleController.reset(drive.getYaw().getRadians());
        drive.setDriveState(DriveState.DRIVER);

    }

    @Override
    public void execute() {

        leftJoystickValues = joysticks.getLeftJoystickValues()
            .shape(Constants.Joystick.MOVE_DEAD_ZONE, Constants.Joystick.TURN_SENSITIVITY)
            .swap()
            .applyAngleDeadzone(Constants.Joystick.ANGLE_DEAD_ZONE);
        rightJoystickValues = joysticks.getRightJoystickValues()
            .shape(Constants.Joystick.TURN_DEAD_ZONE, Constants.Joystick.TURN_SENSITIVITY);

        // leftJoystickValues = joysticks.getLeftOperatorValues()
        //     .shape(Constants.Joystick.MOVE_DEAD_ZONE, Constants.Joystick.TURN_SENSITIVITY)
        //     .swap()
        //     .applyAngleDeadzone(Constants.Joystick.ANGLE_DEAD_ZONE);
        // rightJoystickValues = joysticks.getRightOperatorValues()
        //     .shape(Constants.Joystick.TURN_DEAD_ZONE, Constants.Joystick.TURN_SENSITIVITY);

        chassisSpeeds.vxMetersPerSecond = leftJoystickValues.x * Constants.Drive.MAX_VELOCITY;
        chassisSpeeds.vyMetersPerSecond = leftJoystickValues.y * Constants.Drive.MAX_VELOCITY;


        DriveState turnState = drive.state;
        if(rightJoystickValues.x != 0) turnState = DriveState.DRIVER;


        switch(turnState){
            case DRIVER:
                chassisSpeeds.omegaRadiansPerSecond = -rightJoystickValues.x * Constants.Drive.MAX_ROTATION_VELOCITY;
                break;
            case AIM_TO_AMP:
                

                break;
            case AIM_TO_SPEAKER:
                double goal = Math.atan2(
                    drive.getPose().getY() - Constants.Field.getSpeakerPos().getY(),
                    drive.getPose().getX() - Constants.Field.getSpeakerPos().getX()
                    );
                double currAngle = drive.getYaw().getRadians();
                double angleDiff = goal - drive.getGyroAngle().getRadians();

                goal = angleDiff < (2*Math.PI)/2 ?
                    currAngle + angleDiff :
                    currAngle - ((2*Math.PI) - angleDiff);

                chassisSpeeds.omegaRadiansPerSecond = angleController.calculate(currAngle, goal);
                break;
        }

        chassisSpeeds.omegaRadiansPerSecond = -rightJoystickValues.x * Constants.Drive.MAX_ROTATION_VELOCITY;

        drive.set(chassisSpeeds);

    }

    @Override
    public void end(boolean interrupted) {
        // drive.set(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
