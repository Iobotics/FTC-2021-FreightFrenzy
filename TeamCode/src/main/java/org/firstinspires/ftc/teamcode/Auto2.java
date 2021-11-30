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

@Autonomous(name = "Test Auto")
public class Auto2 extends LinearOpMode {
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

        bot.encoderDrive(1, 10.0, 10.0, 5);
    }
}