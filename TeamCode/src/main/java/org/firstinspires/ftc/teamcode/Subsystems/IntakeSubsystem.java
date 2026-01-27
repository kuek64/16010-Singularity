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
        INTAKE, STOP, REVERSE
    }
    public IntakeState istate;
    public int kState = -1;
    public Timer kickerTimer, kTimer, mTimer;
    private DcMotorEx intake;
    private Servo kicker;
    private Servo gate;
    private boolean kBoolean = false;
    public static double kick = 0;
    public static double set = 0.3;
    public static double open = 0.7;
    public static double close = 0.9;
    public static double intakeWaitTime = 0.1;
    public static double setWaitTime = 0.4;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        kicker = hardwareMap.get(Servo.class, "kicker");
        gate = hardwareMap.get(Servo.class, "gate");

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

    public void kickSequence() {

        if ((kTimer.getElapsedTimeSeconds() > 1.5) && kState == -1) {
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

    public void kick() {
        kicker.setPosition(kick);
    }

    public void set() {
        kicker.setPosition(set);
    }

    public void update() {
        intakeState();
        kickSeries();
    }

    public void open() {
        gate.setPosition(open);
    }

    public void close() {
        gate.setPosition(close);
    }
}
