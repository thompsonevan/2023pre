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

    State desState;
    Timer time;

    String path1 = "paths/output/path1.wpilib.json";
    String path2 = "paths/output/path2.wpilib.json";
    String path3 = "paths/output/path3.wpilib.json";
    String path4 = "paths/output/path4.wpilib.json";
    String path5 = "paths/output/path5.wpilib.json";


    Trajectory trajectory1 = new Trajectory();
    Trajectory trajectory2 = new Trajectory();
    Trajectory trajectory3 = new Trajectory();
    Trajectory trajectory4 = new Trajectory();
    Trajectory trajectory5 = new Trajectory();


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
    
        try {
            Path pathe1 = Filesystem.getDeployDirectory().toPath().resolve(path1);
            Path pathe2 = Filesystem.getDeployDirectory().toPath().resolve(path2);
            Path pathe3 = Filesystem.getDeployDirectory().toPath().resolve(path3);
            Path pathe4 = Filesystem.getDeployDirectory().toPath().resolve(path4);
            Path pathe5 = Filesystem.getDeployDirectory().toPath().resolve(path5);

            trajectory1 = TrajectoryUtil.fromPathweaverJson(pathe1);
            trajectory2 = TrajectoryUtil.fromPathweaverJson(pathe2);
            trajectory3 = TrajectoryUtil.fromPathweaverJson(pathe3);
            trajectory4 = TrajectoryUtil.fromPathweaverJson(pathe4);
            trajectory5 = TrajectoryUtil.fromPathweaverJson(pathe5);


         } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + path1, ex.getStackTrace());
         }

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

    int step;
    TrajectoryConfig config;
    Trajectory test;

    Pose2d[] locations;

    @Override
    public void autonomousInit() {
        // config = new TrajectoryConfig(.1, .05);
        // test = TrajectoryGenerator.generateTrajectory(new Pose2d(0,0,Pigeon.getRotation2d()), 
        //                                               List.of(new Translation2d(.75, 0), new Translation2d(.75, .75), new Translation2d(0, .75)),
        //                                               new Pose2d(0,.75,Rotation2d.fromDegrees(180)),
        //                                               config);

        locations = new Pose2d[]{
            new Pose2d(.75, 0, Rotation2d.fromDegrees(90)),
            new Pose2d(.75, .75, Rotation2d.fromDegrees(180)),
            new Pose2d(0, .75, Rotation2d.fromDegrees(270)),
            new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
        };

        time = new Timer();
        time.reset();
        Pigeon.zeroSensor();
        drivetrain.zero();

        step = 0;
    }

    State goal;
    Pose2d goalPose;


    @Override
    public void autonomousPeriodic() {
        double speed = .1;
        double acc = .05;

        double requiredX = .05;
        double requiredY = .05;
        double requiredTheta = 2;

        switch(step){
            case 0:
                goalPose = locations[step];
                desState = new State(time.get(), speed, acc, goalPose, 1000);
                if (Math.abs(drivetrain.getPose().getX() - desState.poseMeters.getX()) < requiredX 
                    && Math.abs(drivetrain.getPose().getY() - desState.poseMeters.getY()) < requiredY
                    && Math.abs(Math.abs(Pigeon.getRotation2d().getDegrees()) - Math.abs(desState.poseMeters.getRotation().getDegrees())) < requiredTheta){
                    step++;
                    time.reset();
                }
            break;
            case 1:
                goalPose = locations[step];
                desState = new State(time.get(), speed, acc, goalPose, 1000);
                if (Math.abs(drivetrain.getPose().getX() - desState.poseMeters.getX()) < requiredX 
                    && Math.abs(drivetrain.getPose().getY() - desState.poseMeters.getY()) < requiredY
                    && Math.abs(Math.abs(Pigeon.getRotation2d().getDegrees()) - Math.abs(desState.poseMeters.getRotation().getDegrees())) < requiredTheta){
                    step++;
                    time.reset();
                }
            break;
            case 2:
                goalPose = locations[step];
                desState = new State(time.get(), speed, acc, goalPose, 1000);
                if (Math.abs(drivetrain.getPose().getX() - desState.poseMeters.getX()) < requiredX 
                    && Math.abs(drivetrain.getPose().getY() - desState.poseMeters.getY()) < requiredY
                    && Math.abs(Math.abs(Pigeon.getRotation2d().getDegrees()) - Math.abs(desState.poseMeters.getRotation().getDegrees())) < requiredTheta){
                    step++;
                    time.reset();
                }
            break;
            case 3:
                goalPose = locations[step];
                desState = new State(time.get(), speed, acc, goalPose, 1000);
                if (Math.abs(drivetrain.getPose().getX() - desState.poseMeters.getX()) < requiredX 
                    && Math.abs(drivetrain.getPose().getY() - desState.poseMeters.getY()) < requiredY
                    && Math.abs(Math.abs(Pigeon.getRotation2d().getDegrees()) - Math.abs(desState.poseMeters.getRotation().getDegrees())) < requiredTheta){
                    step++;
                    time.reset();
                }
            break;
        }

        drivetrain.runAuto(desState, desState.poseMeters.getRotation());

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
