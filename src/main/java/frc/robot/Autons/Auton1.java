package frc.robot.Autons;

import java.io.IOException;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.sensors.Pigeon;
import frc.robot.subsystems.Drivetrain;
import javax.swing.TransferHandler;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Filesystem;

public class Auton1 extends AutonBase{
    enum AutoState {
        step1,
        step2,
        step3,
        step4,
        end
    }

    public AutoState autoState;

    public Timer timer = new Timer();

    Trajectory trajectory1;
    Trajectory trajectory2;
    Trajectory trajectory3;
    Trajectory trajectory4;

    public Auton1(){
        try {
            trajectory1 = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/output/path1.wpilib.json"));
            trajectory2 = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/output/path2.wpilib.json"));
            trajectory3 = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/output/path3.wpilib.json"));
            trajectory4 = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve("paths/output/path4.wpilib.json"));
        } catch (IOException ex) {

        }

        autoState = AutoState.step1;
        timer.reset();

        desState = new State();
        desState.poseMeters = trajectory1.getInitialPose();
    }

    public void runAuto(){
        double requiredX = .05;
        double requiredY = .05;
        double requiredTheta = 5;

        switch(autoState){
            case step1:
                desState = trajectory1.getStates().get(trajectory1.getStates().toArray().length -1);
                if (Math.abs(Drivetrain.getPose().getX() - desState.poseMeters.getX()) < requiredX 
                    && Math.abs(Drivetrain.getPose().getY() - desState.poseMeters.getY()) < requiredY
                    && Math.abs(Math.abs(Pigeon.getRotation2d().getDegrees()) - Math.abs(desState.poseMeters.getRotation().getDegrees())) < requiredTheta){
                        autoState = AutoState.step2;
                        timer.reset();
                }
            break;
            case step2:
                desState = trajectory2.getStates().get(trajectory1.getStates().toArray().length -1);
                if (Math.abs(Drivetrain.getPose().getX() - desState.poseMeters.getX()) < requiredX 
                    && Math.abs(Drivetrain.getPose().getY() - desState.poseMeters.getY()) < requiredY
                    && Math.abs(Math.abs(Pigeon.getRotation2d().getDegrees()) - Math.abs(desState.poseMeters.getRotation().getDegrees())) < requiredTheta){
                        autoState = AutoState.step3;
                        timer.reset();
                }
            break;
            case step3:
                desState = trajectory3.getStates().get(trajectory1.getStates().toArray().length -1);
                if (Math.abs(Drivetrain.getPose().getX() - desState.poseMeters.getX()) < requiredX 
                    && Math.abs(Drivetrain.getPose().getY() - desState.poseMeters.getY()) < requiredY
                    && Math.abs(Math.abs(Pigeon.getRotation2d().getDegrees()) - Math.abs(desState.poseMeters.getRotation().getDegrees())) < requiredTheta){
                        autoState = AutoState.step4;
                        timer.reset();
                }
            break;
            case step4:
                desState = trajectory4.getStates().get(trajectory1.getStates().toArray().length -1);
                if (Math.abs(Drivetrain.getPose().getX() - desState.poseMeters.getX()) < requiredX 
                    && Math.abs(Drivetrain.getPose().getY() - desState.poseMeters.getY()) < requiredY
                    && Math.abs(Math.abs(Pigeon.getRotation2d().getDegrees()) - Math.abs(desState.poseMeters.getRotation().getDegrees())) < requiredTheta){
                        autoState = AutoState.end;
                        timer.reset();
                }
            break;
            default:
            break;
        }

        SmartDashboard.putNumber("Des X", desState.poseMeters.getX());
        SmartDashboard.putNumber("Des Y", desState.poseMeters.getY());
        SmartDashboard.putNumber("Des Theta", desState.poseMeters.getRotation().getDegrees());

        targetTheta = desState.poseMeters.getRotation();
        desState.curvatureRadPerMeter = 1000;
    }
}
