package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.atan;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Configurable
public class ShooterSubsystem {
    public static double turretOffsetY = 0;
    public static double turretOffsetX  = -4.5;
    public static double fIntercept = 620;
    public static double blueGoalX = 0;
    public static double blueGoalY = 144;
    public static double redGoalX  = 140;
    public static double redGoalY  = 140;
    private DcMotorEx flywheel1 = null;
    private DcMotorEx flywheel2 = null;
    private DcMotorEx turret = null;
    private RGBLight light = null;
    public static double tSlope = 14.9333;
    public static double fSlope = 4.2;
    public static int pos = 0;
    public static int vel = 0;
    public static double p = 200;
    public static double i = 0;
    public static double d = 0;
    public static double f = 18.5;
    public static double tp = -0.0035;
    public static double ti = 0;
    public static double td = 0.00001;
    public static double tf = 0;
    public static int tOffset = 0;
    public static PIDController tpidfController;
    public static PIDController fpidfController;
    public static int turretPos;
    public static double pidf;
    public static double fpid;
    public static double turretAngle;

    public ShooterSubsystem(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        light = new RGBLight(hardwareMap.get(Servo.class, "light"));

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotor.Direction.FORWARD);
        flywheel1.setDirection(DcMotor.Direction.FORWARD);
        flywheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheel1.setVelocityPIDFCoefficients(p,i,d,f);

        tpidfController = new PIDController(tp,ti,td);
    }

    public void setTurretPosition(int pos) {
        tpidfController.setPID(tp,ti,td);
        pidf = tpidfController.calculate(turretPos, pos);
        tpidfController.setTolerance(1);
        turret.setPower(pidf + tf);
    }

    public void setFlywheelVelocity(int vel) {
        flywheel1.setVelocityPIDFCoefficients(p, i, d, f);
        flywheel1.setVelocity(vel);
        flywheel2.setPower(flywheel1.getPower());
    }

    public void update() {
        turretPos = turret.getCurrentPosition();
        ready();
    }

    public void driftAdjust() {
        tOffset += 10;
    }

    public void alignTurret(double x, double y, double heading, boolean blue, Telemetry telemetry) {
            double headingDeg = Math.toDegrees(heading);

            double cos = Math.cos(heading);
            double sin = Math.sin(heading);

            double rotX = turretOffsetX * cos - turretOffsetY * sin;
            double rotY = turretOffsetX * sin + turretOffsetY * cos;

            x += rotX;
            y += rotY;

            double goalX = blue ? blueGoalX : redGoalX;
            double goalY = blue ? blueGoalY : redGoalY;

            double angleToGoal = Math.toDegrees(Math.atan2((goalX - x), (goalY - y)));

            turretAngle = angleToGoal + headingDeg - 90;

            turretAngle = turretAngle % 360;

            int targetTicks = (int) (tSlope * turretAngle);

            final int TURRET_MIN = -1347;
            final int TURRET_MAX = 1347;

            if (targetTicks >= TURRET_MAX || targetTicks <= TURRET_MIN) {
                pos = 0;
            } else {
                pos = targetTicks;
            }

            double distance = Math.hypot(goalX - x, goalY - y);
            vel = (int) (distance * fSlope + fIntercept);

            setFlywheelVelocity(vel);
            setTurretPosition(pos);
    }

    public void ready() {
        if((Math.abs(turret.getCurrentPosition() - pos) < 5)) {
            light.green();
        } else {
            light.red();
        }
    }

    public int getPos() {
        return turret.getCurrentPosition();
    }
    public double getVel() {
        return flywheel1.getVelocity();
    }

    public double getTurretAngle() {
        return turretAngle;
    }

    public double getVelError() {
        return Math.abs(flywheel1.getVelocity() - vel);
    }

    public void telemetry() {
    }
}
