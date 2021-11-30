package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

@TeleOp(name="name here", group="Iterative Opmode")
//@Disabled
//
public class TeleOp2 extends OpMode {

    //boolean to toggle Tank / POV mode.
    boolean Tank = true;
    // Declare OpMode members. (Any and all motors)
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightMotorFront = null;
    private DcMotor rightMotorBack = null;
    private DcMotor leftMotorFront = null;
    private DcMotor leftMotorBack = null;
    private DcMotor carousel = null;
    private DcMotor elevator1 = null;
    private DcMotor intake_back = null;

    BNO055IMU imu;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        //update status of Telemetry
        telemetry.addData("Status", "Initialized");

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        rightMotorFront = hardwareMap.get(DcMotor.class, "right_front");
        rightMotorBack = hardwareMap.get(DcMotor.class, "right_back");
        leftMotorFront = hardwareMap.get(DcMotor.class, "left_front");
        leftMotorBack = hardwareMap.get(DcMotor.class, "left_back");
        intake_back = hardwareMap.get(DcMotor.class, "intake_back");
        carousel = hardwareMap.get(DcMotor.class, "carousel");
        elevator1 = hardwareMap.get(DcMotor.class, "elevator1");

        //Setup the elevator to work with encoders
        elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevator1.setTargetPosition(0);
        elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        //scooper = hardwareMap.get(DcMotor.class, "intakeMotor");
        //rightTopShooter = hardwareMap.get(DcMotor.class "rightTopShooter");
        //leftTopShooter = hardwareMap.get(DcMotor.class "leftTopShooter");
        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        rightMotorFront.setDirection(DcMotor.Direction.FORWARD);
        rightMotorBack.setDirection(DcMotor.Direction.FORWARD);
        leftMotorFront.setDirection(DcMotor.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotor.Direction.REVERSE);
        carousel.setDirection(DcMotor.Direction.FORWARD);
        elevator1.setDirection(DcMotor.Direction.FORWARD);
        intake_back.setDirection(DcMotor.Direction.FORWARD);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Initialized");
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
        runtime.reset();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    boolean manual_reset = false;
    @Override
    public void loop() {
        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        telemetry.addData("time", "%.2f ms", runtime.milliseconds());
        telemetry.addData("heading", formatAngle(angles.angleUnit, angles.firstAngle));
        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return formatDegrees(AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }

    String formatDegrees(double degrees) {
        return String.format(Locale.getDefault(), "%.1f", AngleUnit.DEGREES.normalize(degrees));
    }
}