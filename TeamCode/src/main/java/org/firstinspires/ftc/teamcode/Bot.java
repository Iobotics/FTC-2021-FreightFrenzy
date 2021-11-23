package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Bot {
    private LinearOpMode opMode;
    private DcMotor leftMotorFront; // Port:
    private DcMotor rightMotorFront; // Port:
    private DcMotor leftMotorBack; // Port:
    private DcMotor rightMotorBack; // Port:
    private DcMotor liftMotor; // Port:
    private DcMotor spinMotor;

    private Servo servo;

    // drivetrain motors
    static final double COUNTS_PER_MOTOR_REV_20 = 537.6;    // https://www.andymark.com/products/neverest-orbital-20-gearmotor
    // lift motors
    static final double COUNTS_PER_MOTOR_REV_60 = 1680;    // https://www.andymark.com/products/neverest-classic-60-gearmotor
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4;     // For figuring circumference

    static final double COUNTS_PER_INCH(double COUNTS_PER_MOTOR_REV) {
        return (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                (WHEEL_DIAMETER_INCHES * Math.PI);
    }

    private ElapsedTime runtime = new ElapsedTime();

    public Bot(LinearOpMode opMode) {
        this.opMode = opMode;
        init(opMode.hardwareMap);
    }


    public void init(HardwareMap map) {
        leftMotorFront = map.get(DcMotor.class, "left_front");
        rightMotorFront = map.get(DcMotor.class, "right_front");
        leftMotorBack = map.get(DcMotor.class, "left_back");
        rightMotorBack = map.get(DcMotor.class, "right_back");
        liftMotor = map.get(DcMotor.class, "lift");
        spinMotor = map.get(DcMotor.class, "spin");

        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        spinMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        rightMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        servo = map.get(Servo.class, "claw");
        servo.setDirection(Servo.Direction.REVERSE);
    }


    public int getRightFrontWheelPosition() {
        return rightMotorFront.getCurrentPosition();
    }
    public int getRightBackWheelPosition() {
        return rightMotorBack.getCurrentPosition();
    }
    public int getLeftFrontWheelPosition() { return leftMotorFront.getCurrentPosition(); }
    public int getLeftBackWheelPosition() {
        return leftMotorBack.getCurrentPosition();
    }

    public void setDriveTrain(double left, double right) {
        leftMotorFront.setPower(left);
        rightMotorFront.setPower(right);
        leftMotorBack.setPower(left);
        rightMotorBack.setPower(right);
    }

    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {

        int newFrontLeftTarget;
        int newFrontRightTarget;
        int newBackLeftTarget;
        int newBackRightTarget;

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            // Determine new target position, and pass to motor controller
            newFrontLeftTarget = leftMotorFront.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH(COUNTS_PER_MOTOR_REV_20));
            newFrontRightTarget = rightMotorFront.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH(COUNTS_PER_MOTOR_REV_20));
            newBackLeftTarget = leftMotorBack.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH(COUNTS_PER_MOTOR_REV_20));
            newBackRightTarget = rightMotorBack.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH(COUNTS_PER_MOTOR_REV_20));
            leftMotorFront.setTargetPosition(newFrontLeftTarget);
            rightMotorFront.setTargetPosition(newFrontRightTarget);
            leftMotorBack.setTargetPosition(newBackLeftTarget);
            rightMotorBack.setTargetPosition(newBackRightTarget);


            // Turn On RUN_TO_POSITION
            rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftMotorFront.setPower(Math.abs(speed));
            rightMotorFront.setPower(Math.abs(speed));
            leftMotorBack.setPower(Math.abs(speed));
            rightMotorBack.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (leftMotorFront.isBusy() || rightMotorFront.isBusy() || leftMotorBack.isBusy() || rightMotorBack.isBusy())) {

                // Display it for the driver.
                opMode.telemetry.addData("Path1", "Running to %7d :%7d", newFrontLeftTarget, newFrontRightTarget, newBackLeftTarget, newBackRightTarget);
                opMode.telemetry.addData("Path2", "Running at %7d :%7d",
                        leftMotorFront.getCurrentPosition(),
                        rightMotorFront.getCurrentPosition(),
                        leftMotorBack.getCurrentPosition(),
                        rightMotorBack.getCurrentPosition());
                opMode.telemetry.update();
            }

            // Stop all motion;
            leftMotorFront.setPower(0);
            rightMotorFront.setPower(0);
            leftMotorBack.setPower(0);
            rightMotorBack.setPower(0);

            // Turn off RUN_TO_POSITION
            leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
    }

    public void setLift(double power) {
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setPower(power);
    }

    public int getLiftPosition() {
        return liftMotor.getCurrentPosition();
    }

    public void liftToPosition(double speed,
                               int position,
                               double timeoutS) {

        // Ensure that the opmode is still active
        if (opMode.opModeIsActive()) {
            liftMotor.setTargetPosition(position);


            // Turn On RUN_TO_POSITION
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            liftMotor.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opMode.opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (liftMotor.isBusy())) {
            }
        }
    }

    public void liftRunToPosition() {
        if(opMode.opModeIsActive()) {
            liftMotor.setTargetPosition(liftMotor.getCurrentPosition());
            liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            liftMotor.setPower(1);
        }
    }

    public void servoPosition(double position) {
        servo.setPosition(position);
    }

    public void setSpin(double power) {
        spinMotor.setPower(power);
    }

    public double absRange(double input, double range){
        if (input <= range || input >= -range){
            return input;
        }
        else return 1 * ((Math.abs(input))/ input);
    }
}