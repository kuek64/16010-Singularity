package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import static org.firstinspires.ftc.teamcode.OpModes.Auto.RedFifteenArtifact.autoEndPose;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (name = "Red Far", group = "Autonomous")
public class RedFarSide extends OpMode {
    public static Follower follower;

    public ShooterSubsystem shooter;
    public IntakeSubsystem intake;
    private Timer pathTimer, opmodeTimer;
    private int pathState;
    public int pos;
    public int vel;
    private final Pose bluestartPose = new Pose(62, 10, Math.toRadians(90));
    private final Pose bluescorePose = new Pose(60, 20, Math.toRadians(90));
    private final Pose bluepickup1Pose = new Pose(10.75, 6.75, Math.toRadians(180));
    private final Pose bluepickup3Pose = new Pose(6, 36, Math.toRadians(180));
    private final Pose blueintake3Pose = new Pose(36, 36, Math.toRadians(180));
    private final Pose blueparkPose = new Pose(36, 20, Math.toRadians(90));

    private final Pose startPose = bluestartPose.mirror();
    private final Pose scorePose = bluescorePose.mirror();
    private final Pose pickup1Pose = bluepickup1Pose.mirror();
    private final Pose pickup3Pose = bluepickup3Pose.mirror();
    private final Pose intake3Pose = blueintake3Pose.mirror();
    private final Pose parkPose = blueparkPose.mirror();

    private PathChain score, scorePickup1, grabPickup1, scorePickup3, grabPickup3, leave;

    public void buildPaths() {
        score = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading(), 0.2)
                .build();
        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intake3Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), intake3Pose.getHeading())
                .addPath(new BezierLine(intake3Pose, pickup3Pose))
                .setLinearHeadingInterpolation(intake3Pose.getHeading(), pickup3Pose.getHeading())
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();
        leave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                vel = 1560;
                pos = -135;
                intake.open();
                intake.stop();
                follower.followPath(score);
                setPathState(1);
                break;
            case 1:
                if (pathTimer.getElapsedTimeSeconds() > 1.75) {
                    intake.kickSequence();
                }
                if (pathTimer.getElapsedTimeSeconds() > 3.55) {
                    intake.intake();
                    follower.setMaxPower(1);
                    follower.followPath(grabPickup3, true);
                    setPathState(2);
                }
                break;
            case 2:
                if (pathTimer.getElapsedTimeSeconds() > 2.1) {
                    follower.followPath(scorePickup3, true);
                    setPathState(3);
                }
                break;
            case 3:
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    intake.kickSequence();
                }
                if (pathTimer.getElapsedTimeSeconds() > 3.75) {
                    intake.intake();
                    follower.setMaxPower(1);
                    follower.followPath(grabPickup1, true);
                    setPathState(4);
                }
                break;
            case 4:
                if (pathTimer.getElapsedTimeSeconds() > 2.5) {
                    follower.followPath(scorePickup1, true);
                    setPathState(5);
                }
                break;
            case 5:
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    intake.kickSequence();
                }
                if (pathTimer.getElapsedTimeSeconds() > 3.75) {
                    intake.intake();
                    follower.setMaxPower(1);
                    follower.followPath(grabPickup1, true);
                    setPathState(6);
                }
                break;
            case 6:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(scorePickup1, true);
                    setPathState(7);
                }
                break;
            case 7:
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    intake.kickSequence();
                }
                if (pathTimer.getElapsedTimeSeconds() > 3.75) {
                    intake.intake();
                    follower.setMaxPower(1);
                    follower.followPath(grabPickup1, true);
                    setPathState(8);
                }
                break;
            case 8:
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(scorePickup1, true);
                    setPathState(9);
                }
                break;
            case 9:
                if (pathTimer.getElapsedTimeSeconds() > 1.5) {
                    intake.kickSequence();
                }
                if(pathTimer.getElapsedTimeSeconds() > 4) {
                    pos = 0;
                    shooter.setFlywheelVelocity(0);
                    follower.followPath(leave);
                    setPathState(-1);
                }
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void start() {
        pathTimer.resetTimer();
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    public void loop() {
        follower.update();
        shooter.update();
        intake.update();
        shooter.setTurretPosition(pos);
        shooter.setFlywheelVelocity(vel);
        autonomousPathUpdate();
        telemetry.addData("Path State: ", pathState);
        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("Heading: ", follower.getPose().getHeading());
        telemetry.update();
    }

    public void stop() {
        autoEndPose = follower.getPose();
    }
}
