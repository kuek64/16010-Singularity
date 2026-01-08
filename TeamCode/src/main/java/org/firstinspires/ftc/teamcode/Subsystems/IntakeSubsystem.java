package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class IntakeSubsystem {
    public enum IntakeState {
        INTAKE, STOP, REVERSE, PARTIALINTAKE
    }
    public IntakeState istate;
    public int kState = -1;
    public Timer kickerTimer, kTimer, mTimer;
    private DcMotorEx intake;
    private Servo kicker;
    private DistanceSensor dsensor;
    private DistanceSensor dsensor2;
    private boolean kBoolean = false;
    public static double kick = 0;
    public static double set = 0.3;
    public static double intakeWaitTime = 0.1;
    public static double setWaitTime = 0.25;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        kicker = hardwareMap.get(Servo.class, "kicker");
        dsensor = hardwareMap.get(DistanceSensor.class, "csensor");
        dsensor2 = hardwareMap.get(DistanceSensor.class, "csensor2");

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kickerTimer = new Timer();
        kTimer = new Timer();
        mTimer = new Timer();
    }

    public double getCurrent() {
        return intake.getCurrent(CurrentUnit.AMPS);
    }

    public void intakeState() {
        if(istate == IntakeState.INTAKE) {
            intake.setPower(1);
            istate = IntakeState.INTAKE;
        } else if(istate == IntakeState.STOP) {
            intake.setPower(0);
            istate = IntakeState.STOP;
        } else if(istate == IntakeState.REVERSE) {
            intake.setPower(-1);
            istate = IntakeState.REVERSE;
        } else if(istate == IntakeState.PARTIALINTAKE) {
            intake.setPower(0.15);
        }
    }

    public void switchIntake() {
        if(istate == IntakeState.INTAKE) {
            stop();
            istate = IntakeState.STOP;
        } else if(istate == IntakeState.STOP) {
            intake();
            istate = IntakeState.INTAKE;
        } else {
            intake();
            istate = IntakeState.INTAKE;
        }
    }

    public void setIntakeState(IntakeState state) {
        istate = state;
    }

    public void intake() {
        setIntakeState(IntakeState.INTAKE);
    }

    public void stop() {
        setIntakeState(IntakeState.STOP);
    }

    public void reverse() {
        setIntakeState(IntakeState.REVERSE);
    }

    public void partialintake() {
        setIntakeState(IntakeState.PARTIALINTAKE);
    }

    public void overIntake() {
        if(intake.getCurrent(CurrentUnit.AMPS) > 8) {
            partialintake();
        }
    }

    public void kickSequence() {
        double distance = dsensor.getDistance(DistanceUnit.INCH);
        double distance2 = dsensor2.getDistance(DistanceUnit.INCH);

        if ((distance <= 1 || distance2 <= 1 || kTimer.getElapsedTimeSeconds() > 1.5) && kState == -1) {
            kickerSeriesStart();
        }
    }

    public void kickSequenceTeleOp() {
        if (kState == -1) {
            kickerSeriesStart();
        }
    }

    public void kickerSeriesStart() {
        setKickState(0);
    }

    public void setKickState(int state) {
        kState = state;
        kTimer.resetTimer();
    }

    public void kickSeries() {
        switch (kState) {
            case 0:
                stop();
                setKickState(1);
                break;
            case 1:
                if (kTimer.getElapsedTimeSeconds() > 0) {
                    kick();
                    setKickState(2);
                }
                break;
            case 2:
                if (kTimer.getElapsedTimeSeconds() > setWaitTime) {
                    set();
                    setKickState(3);
                }
                break;
            case 3:
                if (kTimer.getElapsedTimeSeconds() > intakeWaitTime) {
                    intake();
                    setKickState(-1);
                }
                break;
            case -1:
            default:
                break;
        }
    }

    public double getDistance1() {
        return dsensor.getDistance(DistanceUnit.INCH);
    }
    public double getDistance2() {
        return dsensor2.getDistance(DistanceUnit.INCH);
    }

    public void kick() {
        kicker.setPosition(kick);
    }

    public void set() {
        kicker.setPosition(set);
    }

    public void update() {
        intakeState();
        kickSeries();
        overIntake();
    }
}
