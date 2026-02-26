package org.firstinspires.ftc.teamcode.Subsystems;

import static java.lang.Math.atan;

import com.arcrobotics.ftclib.controller.PIDController;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

@Configurable
public class ShooterSubsystem {
    public static double turretOffsetY = 0;
    public static double turretOffsetX  = -4;
    public static double blueGoalX = 6;
    public static double blueGoalY = 144;
    public static double redGoalX  = 138;
    public static double redGoalY  = 144;
    private DcMotorEx flywheel1 = null;
    private DcMotorEx flywheel2 = null;
    private DcMotorEx turret = null;
    private RGBLight light = null;
    public static double tSlope = -5.563;
    public static int pos = 0;
    public static int vel = 0;
    public static double p = .002;
    public static double i = 0;
    public static double d = 0;
    public static double f = 0.000335;
    public static double ff = 0.06;
    public static double tp = 0.005;
    public static double ti = 0;
    public static double td = 0.00001;
    public static double tf = 0;
    public static int tOffset = 0;
    public static PIDController tpidfController;
    public static PIDController fpidfController;
    public static int turretPos;
    public static double pidf;
    public static double fpidf;
    public static double turretAngle;
    public static double timeInAirB = 0.6;
    public static double timeInAirM = 0.0012;
    public static double fIntercept = 720;
    public static double fSlope = 5.6;


    public ShooterSubsystem(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotor.Direction.FORWARD);
        flywheel1.setDirection(DcMotor.Direction.REVERSE);
        flywheel2.setDirection(DcMotor.Direction.REVERSE);

        flywheel1.setVelocityPIDFCoefficients(p,i,d,f);

        tpidfController = new PIDController(tp,ti,td);
        fpidfController = new PIDController(p,i,d);
    }

    public void setTurretPosition(int pos) {
        tpidfController.setPID(tp,ti,td);
        pidf = tpidfController.calculate(turretPos, pos);
        tpidfController.setTolerance(1);
        turret.setPower(pidf + tf);
    }

    public void setFlywheelVelocity(int vel) {
        fpidfController.setPID(p,i,d);
        fpidf = fpidfController.calculate(getVel(), vel);
        flywheel1.setPower(fpidf + ff + (f*vel));
        flywheel2.setPower(fpidf + ff + (f*vel));
    }

    private void setPower(double k) {
        flywheel1.setPower(k);
        flywheel2.setPower(k);
    }

    public void update() {
        turretPos = turret.getCurrentPosition();
    }

    public static int calculateRPM(double xInches, double yInches) {
        double distance = Math.hypot(xInches, yInches);

        double rpm = fIntercept + distance*fSlope;

        return (int) rpm;
    }

    public void alignTurret(double x, double y, double heading, boolean blue, Telemetry telemetry, double magVel, double thetaVel) {
        final int TURRET_MIN = -650;
        final int TURRET_MAX = 1350;

        double headingDeg = Math.toDegrees(heading);

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double rotX = turretOffsetX * cos - turretOffsetY * sin;
        double rotY = turretOffsetX * sin + turretOffsetY * cos;

        x += rotX;
        y += rotY;

        double goalX = blue ? blueGoalX : redGoalX;
        double goalY = blue ? blueGoalY : redGoalY;

        /* ---------------- TURRET ANGLE ---------------- */
        double angleToGoal = Math.toDegrees(Math.atan2(goalX - x, goalY - y));
        turretAngle = angleToGoal + headingDeg - 90;

        double currentAngleDeg = turret.getCurrentPosition() / tSlope;
        double error = turretAngle - currentAngleDeg;

        while (error > 180) error -= 360;
        while (error < -180) error += 360;

        int targetTicks = turret.getCurrentPosition() + (int) (error * tSlope);

        if (targetTicks >= TURRET_MAX || targetTicks <= TURRET_MIN) {
            pos = 0;
        } else {
            pos = targetTicks;
        }

        double distance = Math.hypot(goalX-x, goalY-y);

        double finalRPM = fIntercept + distance*fSlope;

        setFlywheelVelocity((int)(finalRPM));
        setTurretPosition(pos + tOffset);
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
        return Math.abs(getVel() - vel);
    }

    public void telemetry() {
    }
}
