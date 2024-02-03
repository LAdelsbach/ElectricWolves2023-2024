package org.firstinspires.ftc.robotcontroller.internal;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "DistanceRecognition", group = "Iterative Opmode")
@Config
public class DistanceRecognition extends OpMode{
    DcMotor motorFL;
    DcMotor motorFR;
    DcMotor motorBL;
    DcMotor motorBR;


    DistanceSensor left;

    DistanceSensor right;


    double xo;
    double deltaY;

    double angleOfArm;


    int disableTouchSensor;
    int manualClaw;

    public static double maxSpeedMotor = 0.1;

    public static int distanceFromWall = 1;
    public static double slowMultiplier = 2;
    public static double maxTurn = 0.1;


    public static double slowDistance =  40;
    public static double slowConstant = 0.3;


    public DistanceRecognition(){
        super();
    }
    public void init() {
        //motors needs to be configured on the controller phones exactly as they are here to work
        //the following code is used to map our motor variables to the physical motors
        motorFL = hardwareMap.dcMotor.get("FL");
        motorFR = hardwareMap.dcMotor.get("FR");
        motorBL = hardwareMap.dcMotor.get("BL");
        motorBR = hardwareMap.dcMotor.get("BR");

         left = hardwareMap.get(DistanceSensor.class, "left");
         right = hardwareMap.get(DistanceSensor.class, "right");

        //the following code is used to set the initial directions of the motors.
        //normally, the motors on the left should be set to Direction.REVERSE,
        //but testing should resolve any issues with motor directions.
        motorFL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorFR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBL.setDirection(DcMotorSimple.Direction.FORWARD);
        motorBR.setDirection(DcMotorSimple.Direction.REVERSE);
        motorFL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        //TODO: Set angle of Arm to actual value
        angleOfArm = 45;

        disableTouchSensor = 1;
        manualClaw = 1;
    }

    public void loop() {
        controlRobot();
    }
    private void controlRobot() {
        movement();
        //TODO: Make this turn on and off using right bumber
    }
    private void movement(){


        if(!gamepad1.a) {
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            motorFL.setPower(frontLeftPower);
            motorBL.setPower(backLeftPower);
            motorFR.setPower(frontRightPower);
            motorBR.setPower(backRightPower);
        }
        else{
            double smallY = left.getDistance(DistanceUnit.CM);
            boolean isRight = false;

            if(smallY > right.getDistance(DistanceUnit.CM)){
                smallY = right.getDistance(DistanceUnit.CM);
                isRight = true;
            }
            deltaY = left.getDistance(DistanceUnit.CM) - right.getDistance(DistanceUnit.CM);
            deltaY = Math.abs(deltaY);
            deltaY /= smallY * slowConstant;

            double fl = maxSpeedMotor;
            double fr = maxSpeedMotor;
            double bl = maxSpeedMotor;
            double br = maxSpeedMotor;

            if(smallY > slowDistance){
                fl = (smallY-distanceFromWall)/(smallY * slowMultiplier);
                fr = (smallY-distanceFromWall)/(smallY * slowMultiplier);
                bl = (smallY-distanceFromWall)/(smallY * slowMultiplier);
                br = (smallY-distanceFromWall)/(smallY * slowMultiplier);
            }
            double currentTurn = deltaY;
            if(currentTurn < maxTurn){
                currentTurn = maxTurn;
            }

            if(!isRight){
                if(fr + currentTurn > maxSpeedMotor || br +currentTurn> maxSpeedMotor){
                    fl -= currentTurn;
                    bl -= currentTurn;
                }
                else{
                    fr += currentTurn;
                    br += currentTurn;
                }
            }
            else{
                if(fl + currentTurn > maxSpeedMotor || bl +currentTurn> maxSpeedMotor){
                    fr -= currentTurn;
                    br -= currentTurn;
                }
                else{
                    fl += currentTurn;
                    bl += currentTurn;
                }
            }

            motorFL.setPower(fl);
            motorBL.setPower(bl);
            motorFR.setPower(fr);
            motorBR.setPower(br);




        }
        telemetry.addData("diffy", deltaY);
        telemetry.addData("left", left.getDistance(DistanceUnit.CM));
        telemetry.addData("right", right.getDistance(DistanceUnit.CM));
        telemetry.update();


    }


}

