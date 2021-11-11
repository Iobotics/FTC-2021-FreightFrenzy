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

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp", group = "TeleOp")
public class TeleOp extends LinearOpMode {
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

        double servoPosition = 1.0;
        double servoTimeout = 0;

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            // If your number X falls between A and B, and you would like Y to fall between C and D, you can apply the following linear transform:
            // Y = (X-A)/(B-A) * (D-C) + C
            bot.setDriveTrain(-gamepad1.left_stick_y * 0.5, -gamepad1.right_stick_y * 0.5);

            if(gamepad1.dpad_up) {
                bot.setLift(0.25);
            }
            else if (gamepad1.dpad_down) {
                bot.setLift(-0.25);
            }
            else {
                bot.setLift(0);
            }

            if(gamepad1.a && servoTimeout <= runtime.milliseconds()) {
                bot.servoPosition(servoPosition);
                servoPosition = (servoPosition == 0.0) ? 1.0 : 0.0;
                servoTimeout = runtime.milliseconds() + 500;
            }

            Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

            telemetry.addData("cm", "%.2f cm", rangeSensor.getDistance(DistanceUnit.CM));
            telemetry.addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));
            telemetry.addData("lift position", "%d", bot.getLiftPosition());
            telemetry.addData("servo position", "%.2f", (servoPosition == 0.0) ? 1.0 : 0.0);
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