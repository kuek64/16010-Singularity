package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class SuperSimpleFlywheelTest extends OpMode {
    DcMotorEx flywheel1, flywheel2;
    public static double target = 2100;


    @Override
    public void init() {
        flywheel1 = hardwareMap.get(DcMotorEx.class, "flywheel1");
        flywheel2 = hardwareMap.get(DcMotorEx.class, "flywheel2");
        flywheel1.setDirection(DcMotor.Direction.FORWARD);
        flywheel2.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {

        if (gamepad1.a)
            flywheel1.setPower(1);
        else
            flywheel1.setPower(0);

        if (gamepad1.b)
            flywheel2.setPower(1);
        else
            flywheel2.setPower(0);


        telemetry.addData("Flywheel 1 Velocity", flywheel1.getVelocity());
        telemetry.addData("Error", target - flywheel1.getVelocity());
    }
}
