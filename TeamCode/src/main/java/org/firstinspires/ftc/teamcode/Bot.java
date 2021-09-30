package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Bot {
    private LinearOpMode opMode;
    private DcMotor leftMotorFront; //Port: 2.2
    private DcMotor rightMotorFront; //Port: 2.0
    private DcMotor leftMotorBack; //Port: 2.3
    private DcMotor rightMotorBack; //Port: 2.1
    private DcMotor intake; //Port: 1.2
    private DcMotor shooterRight; //Port: 1.0
    private DcMotor shooterLeft; //Port: 1.1

    public Bot(LinearOpMode opMode) {
        this.opMode = opMode;
        init(opMode.hardwareMap);
    }


    public void init(HardwareMap map) {
        leftMotorFront = map.get(DcMotor.class, "left-motor-front");
        rightMotorFront = map.get(DcMotor.class, "right-motor-front");
        leftMotorBack = map.get(DcMotor.class, "left-motor-back");
        rightMotorBack = map.get(DcMotor.class, "right-motor-back");
        intake = map.get(DcMotor.class, "intake");
        shooterRight = map.get(DcMotor.class, "shooter-right");
        shooterLeft = map.get(DcMotor.class, "shooter-left");

        shooterLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    public void setDriveTrain(double left, double right) {
        leftMotorFront.setPower(left);
        rightMotorFront.setPower(right);
        leftMotorBack.setPower(left);
        rightMotorBack.setPower(right);
    }
    public void setIntake(double power) {
        intake.setPower(power);
    }
    public void setShooter(double power) {
        shooterRight.setPower(power);
        shooterLeft.setPower(power);
    }

    public double absRange(double input, double range){
        if (input <= range || input >= -range){
            return input;
        }
        else return 1 * ((Math.abs(input))/ input);
    }
}