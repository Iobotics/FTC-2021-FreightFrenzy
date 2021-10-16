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

        piecePosition block = bot.autoEncoderDrive(0.75, 45, 45, 5);
        telemetry.addData("Block position", block.toString());
        telemetry.update();
    }

    public static double rangeSensorFunction() {
        return rangeSensor.getDistance(DistanceUnit.CM);
    }
}
        /*
        RANGE1 = hardwareMap.i2cDevice.get("range");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

        waitForStart();
        Bot bot = new Bot(this);

        long millis = System.currentTimeMillis();

        while (opModeIsActive()) {
            range1Cache = RANGE1Reader.read(RANGE1_REG_START, RANGE1_READ_LENGTH);

            telemetry.addData("Ultra Sonic", range1Cache[0] & 0xFF);
            telemetry.addData("ODS", range1Cache[1] & 0xFF);
        }
        */