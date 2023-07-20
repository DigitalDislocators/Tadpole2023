package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPRamseteCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Drive;

public class FollowTrajectory extends PPRamseteCommand {

    public FollowTrajectory(String trajectory, boolean reversed, Drive drive) {
        super(
            PathPlanner.loadPath(trajectory, AutoConstants.maxVelMetersPerSecond, AutoConstants.maxAccelMetersPerSecondSq, reversed),
            drive::getPose,
            AutoConstants.kRamseteCotroller,
            new SimpleMotorFeedforward(AutoConstants.ksVolts, AutoConstants.kvVoltSecondsPerMeter, AutoConstants.kaVoltSecondsSquaredPerMeter),
            AutoConstants.kDriveKinematics,
            drive::getWheelSpeeds,
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            drive::driveVolts,
            true,
            drive
        );

        addRequirements(drive);
    }
    
}
