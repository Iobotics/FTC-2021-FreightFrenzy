package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Electric Boogaloo", group = "TeleOp")
public class TeleOp2 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    BNO055IMU imu;

    @Override
    public void runOpMode() throws InterruptedException {
        Bot bot = new Bot(this);
        Rev2mDistanceSensor rangeSensor = hardwareMap.get(Rev2mDistanceSensor.class, "sensor_range");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        int liftBottom = bot.getLiftPosition();
        int liftTop = liftBottom + (int) (10 * ((1680 * 1.0) / (4.0 * Math.PI)));

        double liftTimeout = 0;

        boolean runToPosition = false;
        double runToPositionTimeout = 0;

        double servoPosition = 0.75;
        double servoTimeout = 0;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // If your number X falls between A and B, and you would like Y to fall between C and D, you can apply the following linear transform:
            // Y = (X-A)/(B-A) * (D-C) + C
            bot.setDriveTrain(-gamepad1.left_stick_y * 0.75, -gamepad1.right_stick_y * 0.75);

            if(gamepad1.left_bumper) {
                bot.setLift(0.4);
            }
            else if (gamepad1.left_trigger > 0.2) {
                bot.setLift(-0.4);
            }
            else {
                bot.setLift(0);
            }

            if(gamepad1.a && servoTimeout <= runtime.milliseconds()) {
                bot.servoPosition(servoPosition);
                servoPosition = (servoPosition == 0.1625) ? 0.75 : 0.1625;
                servoTimeout = runtime.milliseconds() + 250;
            }

            if(gamepad1.right_trigger > 0.2 && servoTimeout <= runtime.milliseconds()) {
                bot.liftToPosition(0.5, liftBottom, 10);
                servoTimeout = runtime.milliseconds() + 250;
            }
            else if(gamepad1.right_bumper && servoTimeout <= runtime.milliseconds()) {
                bot.liftToPosition(0.5, liftTop, 10);
                servoTimeout = runtime.milliseconds() + 250;
            }

            if(gamepad1.y) {
                bot.setSpin(1);
            }
            else if(gamepad1.b) {
                bot.setSpin(-1);
            }
            else {
                bot.setSpin(0);
            }

            if(gamepad1.x && runToPositionTimeout <= runtime.milliseconds()) {
                if (runToPosition) {
                    runToPosition = false;
                    bot.liftEndRunToPosition();
                } else {
                    runToPosition = true;
                    bot.liftStartRunToPosition();
                }
                runToPositionTimeout = runtime.milliseconds() + 250;
            }

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addData("time", "%.2f ms", runtime.milliseconds());
            telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));
            telemetry.addData("lift position", bot.getLiftPosition());
            telemetry.addData("left front wheel position", bot.getLeftFrontWheelPosition());
            telemetry.addData("left back wheel position", bot.getLeftBackWheelPosition());
            telemetry.addData("right front wheel position", bot.getRightFrontWheelPosition());
            telemetry.addData("right back wheel position", bot.getRightBackWheelPosition());
            telemetry.addData("servo position", "%.2f", (servoPosition == 0.1625) ? 0.75 : 0.1625);
            telemetry.addData("servo timeout?", "%.2f", servoTimeout);
            telemetry.addData("runToPosition?", runToPosition);
            telemetry.addData("servo timeout?", "%.2f", runToPositionTimeout);
            telemetry.update();
        }
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }

}