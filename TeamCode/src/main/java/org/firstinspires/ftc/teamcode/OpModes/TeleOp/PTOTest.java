package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "PTO Test", group = "TeleOp")
public class PTOTest extends OpMode {
    private DcMotorEx leftBack;
    private DcMotorEx rightBack;
    private Servo leftPTO;
    private Servo rightPTO;

    public static double engage = 0;
    public static double disengage = 0.25;


    public void init() {
        leftBack = hardwareMap.get(DcMotorEx.class, "lb");
        rightBack = hardwareMap.get(DcMotorEx.class, "rb");
        leftPTO = hardwareMap.get(Servo.class, "leftPTO");
        rightPTO = hardwareMap.get(Servo.class, "rightPTO");

        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);

        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    public void loop() {
        if(gamepad1.a) {
            rightBack.setPower(1);
            leftBack.setPower(1);
        } else {
            rightBack.setPower(0);
            leftBack.setPower(0);
        }

        if(gamepad1.b) {
            rightBack.setPower(-1);
            leftBack.setPower(-1);
        } else {
            rightBack.setPower(0);
            leftBack.setPower(0);
        }

        if(gamepad1.x) {
            rightPTO.setPosition(engage);
            leftPTO.setPosition(engage);
        }

        if(gamepad1.y) {
            rightPTO.setPosition(disengage);
            leftPTO.setPosition(disengage);
        }
    }
}
