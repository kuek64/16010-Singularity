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
    public static double blueGoalX = 2.5;
    public static double blueGoalY = 144;
    public static double redGoalX  = 144;
    public static double redGoalY  = 136;
    private DcMotorEx flywheel1 = null;
    private DcMotorEx flywheel2 = null;
    private DcMotorEx turret = null;
    private RGBLight light = null;
    public static double tSlope = -5.563;
    public static int pos = 0;
    public static int vel = 0;
    public static double p = 400;
    public static double i = 0;
    public static double d = 0;
    public static double f = 14.5;
    public static double tp = 0.005;
    public static double ti = 0;
    public static double td = 0.00001;
    public static double tf = 0;
    public static int tOffset = 0;
    public static double EFFICIENCY = 1.425;
    public static double goalHeight = 39;
    public static PIDController tpidfController;
    public static PIDController fpidfController;
    public static int turretPos;
    public static double pidf;
    public static double fpid;
    public static double turretAngle;
    public static double EFFICIENCY_SLOPE = 0.01;    // per meter
    public static double MIN_EFFICIENCY = 1;
    public static double DISTANCE_RPM_GAIN = 140;  // RPM per meter
    public static double timeInAirB = 0.6;
    public static double timeInAirM = 0.0012;


    public ShooterSubsystem(HardwareMap hardwareMap) {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");

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
    }

    public static int calculateRPM(double xInches, double yInches, double goalHeightInches, double shooterHeightInches) {
        // ---- Unit conversion ----
        double x = xInches * 0.0254;
        double y = yInches * 0.0254;
        double deltaZ = (goalHeightInches - shooterHeightInches) * 0.0254;

        // ---- Horizontal distance ----
        double d = Math.sqrt(x * x + y * y);

        // ---- Safety check ----
        if (d <= deltaZ) {
            return 0;
        }

        // ---- Projectile velocity (45° hood) ----
        double velocity = Math.sqrt((9.81 * d * d) / (d - deltaZ));

        // ---- Velocity -> RPM (96mm wheel, 6mm compression => 0.042m radius) ----
        double rpm = (60.0 / (2.0 * Math.PI)) * (velocity / 0.042);

        // ---- Distance-dependent efficiency ----
        double efficiency = EFFICIENCY - (EFFICIENCY_SLOPE * d);
        efficiency = Math.max(MIN_EFFICIENCY, efficiency);

        rpm /= efficiency;

        // ---- Long-range compensation ----
        rpm += DISTANCE_RPM_GAIN * d;

        return (int) rpm;
    }

    public void alignTurret(double x, double y, double heading, boolean blue, Telemetry telemetry, double magVel, double thetaVel) {
        final double GOAL_HEIGHT = 44;
        final double ROBOT_HEIGHT = 16;
        final int TURRET_MIN = -650;
        final int TURRET_MAX = 1250;

        double headingDeg = Math.toDegrees(heading);

        double cos = Math.cos(heading);
        double sin = Math.sin(heading);

        double rotX = turretOffsetX * cos - turretOffsetY * sin;
        double rotY = turretOffsetX * sin + turretOffsetY * cos;

        x += rotX;
        y += rotY;

        double goalX = blue ? blueGoalX : redGoalX;
        double goalY = blue ? blueGoalY : redGoalY;

        /* -------- INITIAL RPM ESTIMATE (for time in air) -------- */
        double dx = goalX - x;
        double dy = goalY - y;

        double rpmEstimate = calculateRPM(dx, dy, GOAL_HEIGHT, ROBOT_HEIGHT);

        /* ---------------- TIME IN AIR ---------------- */
        double timeInAir = timeInAirM * rpmEstimate - timeInAirB;

        /* ---------------- VELOCITY LEAD ---------------- */
        double vx = magVel * Math.cos(thetaVel);
        double vy = magVel * Math.sin(thetaVel);

        x += vx * timeInAir;
        y += vy * timeInAir;

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

        /* -------- FINAL RPM (MATCHES LED POSITION) -------- */
        double finalDx = goalX - x;
        double finalDy = goalY - y;

        double finalRPM = calculateRPM(finalDx, finalDy, GOAL_HEIGHT, ROBOT_HEIGHT);

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
        return Math.abs(flywheel1.getVelocity() - vel);
    }

    public void telemetry() {
    }
}
