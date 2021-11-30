package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    public static Rev2mDistanceSensor rangeSensor;
    public enum piecePosition {
        close,
        medium,
        far
    };
    BNO055IMU imu;
    private DcMotor elevator1 = null;
    private DcMotor intake_back = null;
    private DcMotor rightMotorFront = null;
    private DcMotor rightMotorBack = null;
    private DcMotor leftMotorFront = null;
    private DcMotor leftMotorBack = null;


    @Override
    public void runOpMode() throws InterruptedException {

        Bot bot = new Bot(this);
        elevator1 = hardwareMap.get(DcMotor.class, "elevator1");
        rangeSensor = hardwareMap.get(Rev2mDistanceSensor.class, "sensor_range");

        elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator1.setTargetPosition(0);
        elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator1.setDirection(DcMotor.Direction.FORWARD);
        rightMotorFront = hardwareMap.get(DcMotor.class, "right_front");
        rightMotorBack = hardwareMap.get(DcMotor.class, "right_back");
        leftMotorFront = hardwareMap.get(DcMotor.class, "left_front");
        leftMotorBack = hardwareMap.get(DcMotor.class, "left_back");
        rightMotorFront.setDirection(DcMotor.Direction.FORWARD);
        rightMotorBack.setDirection(DcMotor.Direction.FORWARD);
        leftMotorFront.setDirection(DcMotor.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotor.Direction.REVERSE);

        intake_back = hardwareMap.get(DcMotor.class, "intake_back");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        waitForStart();
        runtime.reset();

//        bot.encoderDrive(1, 10.0, 10.0, 2);
        while(runtime.seconds() < 7)
        {
            elevator1.setTargetPosition(7000);
            elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
//        while (runtime.seconds() < 10
//        ) {
//            intake_back.setPower(-1.0);
//            intake_back.setDirection(DcMotor.Direction.REVERSE);
//        }
        while(runtime.seconds() < 7.6)
        {
            leftMotorBack.setPower(1.0);
            leftMotorFront.setPower(1.0);
            rightMotorBack.setPower(-1.0);
            rightMotorFront.setPower(-1.0);
        }
        while(runtime.seconds() < 9) {
            leftMotorBack.setPower(1.0);
            leftMotorFront.setPower(1.0);
            rightMotorBack.setPower(1.0);
            rightMotorFront.setPower(1.0);
        }
        //wait(6000);


        /*
        double distanceAvg = 0;
        for(int i = 0; i < 50; i++) {
            distanceAvg += rangeSensorFunction();
        }
        distanceAvg = distanceAvg / 50;
        piecePosition block = blockPosition(distanceAvg);
        telemetry.addData("Average Distance", "%.2f", distanceAvg);
        telemetry.addData("Position", "%s", block.toString());
        telemetry.update();

        bot.encoderDrive(1, 30, 30, 30);

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
        double startAngle = angle(angles.angleUnit, angles.firstAngle);
        double targetAngle = startAngle + (90 - 10);
        double currentAngle = angle(angles.angleUnit, angles.firstAngle);

        while(Math.abs(currentAngle) < targetAngle) {
            bot.setDriveTrain(0.5,0);
            angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.YXZ, AngleUnit.DEGREES);
            currentAngle = angle(angles.angleUnit, angles.firstAngle);

            telemetry.addData("Start Angle: ", "%.2f", startAngle);
            telemetry.addData("Target Angle: ", "%.2f", targetAngle);
            telemetry.addData("Current Angle: ", "%.2f", currentAngle);
            telemetry.addData("Loop? ", "%b", (currentAngle < targetAngle));
            telemetry.update();
        }
        bot.setDriveTrain(0,0);

        bot.encoderDrive(1, 18, 18, 30);
        if(block == piecePosition.far)
        {
            elevator1.setTargetPosition(0);
        }
        if(block == piecePosition.medium)
        {
            elevator1.setTargetPosition(7000);
        }
        if(block == piecePosition.far)
        {
            elevator1.setTargetPosition(14000);
        }
        elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        */


    }

    public static double rangeSensorFunction() {
        return rangeSensor.getDistance(DistanceUnit.INCH);
    }

    piecePosition blockPosition(double position) {
        if(position < 8) {
            return piecePosition.close;
        }
        else if(position < 14) {
            return piecePosition.medium;
        }
        else {
            return piecePosition.far;
        }
    }

    double angle(AngleUnit angleUnit, double angle) {
        return AngleUnit.DEGREES.normalize(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }
}