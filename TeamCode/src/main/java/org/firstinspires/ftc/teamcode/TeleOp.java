package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        Bot bot = new Bot(this);
        Rev2mDistanceSensor rangeSensor = hardwareMap.get(Rev2mDistanceSensor.class, "sensor_range");

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            bot.setDriveTrain(-gamepad1.left_stick_y, -gamepad1.right_stick_y);

            // telemetry.addData("time", "%.2f cm", runtime);
            telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.update();
        }
    }
}