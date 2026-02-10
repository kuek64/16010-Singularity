package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous (name = "Red Close", group = "Autonomous")
public class RedFifteenArtifact extends OpMode {
    public static Follower follower;
    public static Pose autoEndPose;

    public ShooterSubsystem shooter;
    public IntakeSubsystem intake;

    private Timer pathTimer, opmodeTimer;

    private int pathState;
    public int pos;
    public int vel;

    private final Pose bluestartPose = new Pose(29.5, 127, Math.toRadians(180));
    private final Pose bluescorePose = new Pose(60, 83.5, Math.toRadians(180));
    private final Pose bluepickup1Pose = new Pose(12, 83.5, Math.toRadians(180));
    private final Pose bluepickup2Pose = new Pose(8, 60, Math.toRadians(180));
    private final Pose bluesetUpPose = new Pose(42, 60, Math.toRadians(180));
    private final Pose blueemptyPose = new Pose(15, 72, Math.toRadians(180));
    private final Pose blueintake2Pose = new Pose(72, 60, Math.toRadians(180));
    private final Pose bluepickup3Pose = new Pose(6, 36, Math.toRadians(180));
    private final Pose blueintake3Pose = new Pose(72, 30, Math.toRadians(180));
    private final Pose blueintake4Pose = new Pose(6, 60, Math.toRadians(270));
    private final Pose bluepickup4Pose = new Pose(4, -12, Math.toRadians(270));
    private final Pose blueleavePose = new Pose(50, 73.5, Math.toRadians(135));

    private final Pose startPose = bluestartPose.mirror();
    private final Pose scorePose = bluescorePose.mirror();
    private final Pose pickup1Pose = bluepickup1Pose.mirror();
    private final Pose pickup2Pose = bluepickup2Pose.mirror();
    private final Pose setUpPose = bluesetUpPose.mirror();
    private final Pose emptyPose = blueemptyPose.mirror();
    private final Pose intake2Pose = blueintake2Pose.mirror();
    private final Pose pickup3Pose = bluepickup3Pose.mirror();
    private final Pose intake3Pose = blueintake3Pose.mirror();
    private final Pose intake4Pose = blueintake4Pose.mirror();
    private final Pose pickup4Pose = bluepickup4Pose.mirror();
    private final Pose leavePose = blueleavePose.mirror();



    private Path scorePreload;
    private PathChain scorePickup1, scorePickup2, scorePickup3, grabPickup1, grabPickup2, grabPickup3, grabPickup4, scorePickup4, leave;

    public void buildPaths() {
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup2 = follower.pathBuilder()
                .addPath( new BezierCurve(scorePose, intake2Pose, pickup2Pose))
                .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                .addPath(new BezierCurve(pickup2Pose, setUpPose, emptyPose))
                .setConstantHeadingInterpolation(emptyPose.getHeading())
                .build();

        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(emptyPose, scorePose))
                .setLinearHeadingInterpolation(emptyPose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, intake3Pose, pickup3Pose))
                .setConstantHeadingInterpolation(pickup2Pose.getHeading())
                .build();

        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3Pose, scorePose))
                .setLinearHeadingInterpolation(pickup3Pose.getHeading(), scorePose.getHeading())
                .build();

        grabPickup4 = follower.pathBuilder()
                .addPath(new BezierCurve(scorePose, intake4Pose, pickup4Pose))
                .setConstantHeadingInterpolation(pickup4Pose.getHeading())
                .build();

        scorePickup4 = follower.pathBuilder()
                .addPath(new BezierLine(pickup4Pose, scorePose))
                .setLinearHeadingInterpolation(intake4Pose.getHeading(), scorePose.getHeading())
                .build();

        leave = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, leavePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                vel = 1220;
                pos = 250;
                follower.setMaxPower(1);
                follower.followPath(scorePreload);
                intake.stop();
                intake.open();
                setPathState(1);
                break;
            case 1:
                if(pathTimer.getElapsedTimeSeconds() > 1.75) {
                    intake.kickSequence();
                }
                if(pathTimer.getElapsedTimeSeconds() > 3.5) {
                    intake.intake();
                    follower.followPath(grabPickup1,true);
                    setPathState(2);
                }
                break;
            case 2:
                if(pathTimer.getElapsedTimeSeconds() > 1.4) {
                    follower.followPath(scorePickup1,true);
                    intake.stop();
                    intake.open();
                    setPathState(3);
                }
                break;
            case 3:
                if(pathTimer.getElapsedTimeSeconds() > 1) {
                    intake.kickSequence();
                }
                if(pathTimer.getElapsedTimeSeconds() > 3.4) {
                    intake.intake();
                    follower.setMaxPower(1);
                    follower.followPath(grabPickup2,true);
                    setPathState(4);
                }
                break;
            case 4:
                if(pathTimer.getElapsedTimeSeconds() > 3.05) {
                    follower.followPath(scorePickup2, true);
                    intake.stop();
                    intake.open();
                    setPathState(5);
                }
                break;
            case 5:
                if(pathTimer.getElapsedTimeSeconds() > 1.65) {
                    intake.kickSequence();
                }
                if(pathTimer.getElapsedTimeSeconds() > 3.35) {
                    intake.intake();
                    follower.setMaxPower(1);
                    follower.followPath(grabPickup3,true);
                    setPathState(6);
                }
                break;
            case 6:
                if(pathTimer.getElapsedTimeSeconds() > 2.25) {
                    follower.followPath(scorePickup3, true);
                    setPathState(7);
                }
                break;
            case 7:
                if(pathTimer.getElapsedTimeSeconds() > 1.75) {
                    intake.kickSequence();
                }
                if(pathTimer.getElapsedTimeSeconds() > 4.3) {
                    intake.intake();
                    follower.setMaxPower(0.85);
                    follower.followPath(grabPickup4,true);
                    setPathState(8);
                }
                break;
            case 8:
                if(pathTimer.getElapsedTimeSeconds() > 3.05) {
                    follower.followPath(scorePickup4, true);
                    intake.stop();
                    intake.open();
                    setPathState(9);
                }
                break;
            case 9:
                if(pathTimer.getElapsedTimeSeconds() > 2.65) {
                    intake.kickSequence();
                }
                if(pathTimer.getElapsedTimeSeconds() > 4.15) {
                    setPathState(10);
                }
                break;
            case 10:
                pos = 0;
                follower.followPath(leave);
                setPathState(-1);
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
        shooter.setTurretPosition(pos);
        shooter.setFlywheelVelocity(vel);
        intake.update();
        autonomousPathUpdate();
        telemetry.addData("Path State: ", pathState);
        telemetry.addData("X: ", follower.getPose().getX());
        telemetry.addData("Y: ", follower.getPose().getY());
        telemetry.addData("Heading: ", follower.getPose().getHeading());
        telemetry.addData("Intake State: ", intake.istate);
        telemetry.update();
    }

    public void stop() {
        autoEndPose = follower.getPose();
    }
}