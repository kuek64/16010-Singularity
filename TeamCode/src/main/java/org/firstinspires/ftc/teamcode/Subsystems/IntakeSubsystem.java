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

    public enum GateState {
        OPEN, CLOSE
    }
    public IntakeState istate;
    public GateState gstate;
    public int kState = -1;
    public Timer kickerTimer, kTimer, mTimer;
    private DcMotorEx intake;
    private Servo kicker;
    private Servo gate;
    private boolean kBoolean = false;
    public static double kick = 0.45;
    public static double set = 0;
    public static double open = 0.7;
    public static double close = 0.9;
    public static double kickWaitTime = 0.4;
    public static double intakeWaitTime = 0.1;
    public static double setWaitTime = 1.3;

    public IntakeSubsystem(HardwareMap hardwareMap) {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        kicker = hardwareMap.get(Servo.class, "kicker");
        gate = hardwareMap.get(Servo.class, "gate");

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

    public void gateState() {
        if(gstate == GateState.OPEN) {
            gate.setPosition(open);
            gstate = GateState.OPEN;
        } else if(gstate == GateState.CLOSE) {
            gate.setPosition(close);
            gstate = GateState.CLOSE;
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

    public void switchGate() {
        if(gstate == GateState.OPEN) {
            close();
            gstate = GateState.CLOSE;
        } else if(gstate == GateState.CLOSE) {
            open();
            gstate = GateState.OPEN;
        } else {
            open();
            gstate = GateState.OPEN;
        }
    }

    public void setIntakeState(IntakeState state) {
        istate = state;
    }
    public void setGateState(GateState state) {
        gstate = state;
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

        if ((kTimer.getElapsedTimeSeconds() > 1) && kState == -1) {
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
                open();
                intake();
                if (kTimer.getElapsedTimeSeconds() > 0.05) {
                    setKickState(1);
                }
                break;
            case 1:
                if (kTimer.getElapsedTimeSeconds() > kickWaitTime) {
                    kick();
                    setKickState(2);
                }
                break;
            case 2:
                if(kTimer.getElapsedTimeSeconds() < 0.4) {
                    kick();
                }
                if((kTimer.getElapsedTimeSeconds() > 0.4) && (kTimer.getElapsedTimeSeconds() < 1)) {
                    set();
                }
                if((kTimer.getElapsedTimeSeconds() > 1) && (kTimer.getElapsedTimeSeconds() < setWaitTime)) {
                    kick();
                }
                if (kTimer.getElapsedTimeSeconds() > setWaitTime) {
                    set();
                    setKickState(3);
                }
                break;
            case 3:
                set();
                if (kTimer.getElapsedTimeSeconds() > intakeWaitTime) {
                    close();
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
        gateState();
        kickSeries();
    }

    public void open() {
        setGateState(GateState.OPEN);
    }

    public void close() {
        setGateState(GateState.CLOSE);
    }
}
