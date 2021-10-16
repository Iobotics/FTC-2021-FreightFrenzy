package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    public static Rev2mDistanceSensor rangeSensor;
    public enum piecePosition {
        close,
        medium,
        far
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Bot bot = new Bot(this);
        rangeSensor = hardwareMap.get(Rev2mDistanceSensor.class, "sensor_range");

        waitForStart();
        runtime.reset();

        /*
        piecePosition block = bot.autoEncoderDrive(0.75, 45, 45, 5);
        telemetry.addData("Block position", block.toString());
        telemetry.update();
        */

        bot.encoderDrive(1, 26, 26, 30);
        double distanceAvg = 0;
        for(int i = 0; i < 50; i++) {
            distanceAvg += rangeSensorFunction();
        }
        distanceAvg = distanceAvg / 50;
        telemetry.addData("Average Distance", "%.2f", distanceAvg);
        telemetry.update();
        wait(5000);
    }

    public static double rangeSensorFunction() {
        return rangeSensor.getDistance(DistanceUnit.INCH);
    }
}