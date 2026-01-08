package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

@Configurable
@TeleOp(name = "Intake Test", group = "TeleOp")
public class IntakeTest extends OpMode {
    IntakeSubsystem intake;
    public void init() {
        intake = new IntakeSubsystem(hardwareMap);
    }

    public void loop() {
        intake.update();

        if(gamepad1.a) {
            intake.kickSequence();
        }

        telemetry.addData("Distance Sensor 1: ", intake.getDistance1());
        telemetry.addData("Distance Sensor 2: ", intake.getDistance2());
    }
}
