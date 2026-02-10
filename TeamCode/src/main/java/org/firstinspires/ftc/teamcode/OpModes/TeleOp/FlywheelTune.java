package org.firstinspires.ftc.teamcode.OpModes.TeleOp;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.ShooterSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.IntakeSubsystem;

@Configurable
@TeleOp(name = "Flywheel Tuning", group = "TeleOp")
public class FlywheelTune extends OpMode {
    public TelemetryManager teleM;
    ShooterSubsystem shooter;
    IntakeSubsystem intake;
    public static int vel = 0;
    private double fError;
    public void init() {
        shooter = new ShooterSubsystem(hardwareMap);
        intake = new IntakeSubsystem(hardwareMap);

        teleM = PanelsTelemetry.INSTANCE.getTelemetry();
    }

    public void loop() {
        intake.update();
        shooter.update();

        if(gamepad1.rightBumperWasPressed()) {
            vel+=100;
            gamepad1.rumble(100);
        } else if(gamepad1.leftBumperWasPressed()) {
            vel-=100;
            gamepad1.rumble(100);
        }

        if(gamepad1.aWasPressed()) {
            intake.kickSequence();
        }

        if(gamepad1.xWasPressed()) {
            intake.switchIntake();
        }

        shooter.setFlywheelVelocity(vel);

        fError = Math.abs(vel-shooter.getVel());

        teleM.addData("Velocity Target: ", vel);
        teleM.addData("Flywheel Error: ", fError);
        teleM.addData("Velocity: ", shooter.getVel());
        teleM.update();
    }
}
