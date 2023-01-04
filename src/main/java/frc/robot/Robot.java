package frc.robot;

import org.hotutilites.hotlogger.HotLogger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrajectoryParameterizer.TrajectoryGenerationException;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Autons.Auton1;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.*;

import java.io.IOException;
import java.nio.file.Path;
import java.util.List;

import javax.swing.text.StyleContext.SmallAttributeSet;

public class Robot extends TimedRobot {
    private TeleopCommander teleopCommander;
    private Drivetrain drivetrain;
    private Pigeon pigeon;
    private AutonCommader autonCommader;
    private Auton1 auton;

    @Override
    public void robotInit() {
        HotLogger.Setup("Theta", "Left Front Absolute", "Left Front Assumed",
        "Right Front Absolute", "Right Front Assumed",
        "Left Rear Absolute", "Left Rear Assumed",
        "Right Rear Absolute", "Right Rear Assumed",
        "TargetX", "TargetY", "TargetTheta", "Robot State Theta", "poseX", "poseY");

        teleopCommander = new TeleopCommander();
        pigeon = new Pigeon();
        drivetrain = new Drivetrain();
        autonCommader = new AutonCommader();
        auton = new Auton1();
    }

    @Override
    public void robotPeriodic() {
    }

    @Override
    public void disabledInit() {
        drivetrain.zero();
    }

    @Override
    public void disabledPeriodic() {
        drivetrain.disabled();
    }

    @Override
    public void autonomousInit() {
        Pigeon.zeroSensor();
        drivetrain.zero();
        autonCommader.initAuton(auton);
    }

    @Override
    public void autonomousPeriodic() {
        pigeon.enabledAction(teleopCommander);
        autonCommader.runAuto();
        drivetrain.autonAction(autonCommader);
    }

    @Override
    public void teleopInit() {
        drivetrain.zero();
        Pigeon.zeroSensor();
    }

    @Override
    public void teleopPeriodic() {
        pigeon.enabledAction(teleopCommander);
        drivetrain.teleAction(teleopCommander);
    }
}
