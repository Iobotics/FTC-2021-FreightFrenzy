package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Basic: Iterative OpMode", group="Iterative Opmode")
//@Disabled
//
public class LilyOpThing extends OpMode {

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
        // Setup a variables to add to telemetry
        double leftPower = 0;
        double rightPower = 0;

        //detect if a manual override of the elevator is being attempted
        if(gamepad1.a && gamepad1.b && gamepad1.x && gamepad1.y) {
            if ((gamepad1.left_trigger > .7) && (gamepad1.right_trigger == 0) && !gamepad1.right_bumper && !gamepad1.left_bumper && gamepad1.left_stick_y!=0) {
                //enable manual control of the elevator using left stick.
                elevator1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                elevator1.setPower(gamepad1.left_stick_y);
                manual_reset = true;
            }
        }
        else
        {
            //enable Tank / POV mode
            if(Tank) {

                leftPower = -gamepad2.left_stick_y;
                rightPower = -gamepad2.right_stick_y;
            } else {
                double drive = -gamepad2.left_stick_y;
                double turn  =  gamepad2.right_stick_x;
                leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
                rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;
            }
            //set Motors to their corresponding powers
            rightMotorFront.setPower(rightPower);
            rightMotorBack.setPower(rightPower);
            leftMotorFront.setPower(leftPower);
            leftMotorBack.setPower(leftPower);

            //control the intake motor
            if (gamepad1.left_trigger>0) {
                intake_back.setPower(1.0);
                intake_back.setDirection(DcMotor.Direction.FORWARD);
            } else if (gamepad1.right_trigger>0) {
                intake_back.setPower(1.0);
                intake_back.setDirection(DcMotor.Direction.REVERSE);
            } else {
                intake_back.setPower(0);
            }

            //control the carousel
            if (gamepad1.b) {
                carousel.setPower(0.7);
            } else {
                carousel.setPower(0);
            }

            //set the elevator to different heights based on dpad positions
            if (gamepad1.dpad_left) {
                elevator1.setTargetPosition(14000);
            }
            if (gamepad1.dpad_up) {
                elevator1.setTargetPosition((14000*2)/3);
            }
            if (gamepad1.dpad_right) {
                elevator1.setTargetPosition(14000/3);
            }
            if (gamepad1.dpad_down) {
                elevator1.setTargetPosition(0);
            }
            //set elevator to max/min height with bumpers
            if(gamepad1.right_bumper)
            {
                elevator1.setTargetPosition(14000);
            }
            if(gamepad1.left_bumper) {
                elevator1.setTargetPosition(0);
            }
            if (manual_reset) {
                elevator1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                elevator1.setTargetPosition(0);
                manual_reset = false;
            }
            elevator1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevator1.setPower(1);
        }


        // Show the elapsed game time and wheel power.
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}