package org.firstinspires.ftc.robotcontroller.internal;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@TeleOp
public class DistanceSensorTest extends LinearOpMode {
    DistanceSensor left;
    DistanceSensor right;


    @Override
    public void runOpMode() {
        // Get the distance sensor and motor from hardwareMap
        left = hardwareMap.get(DistanceSensor.class, "left");
        right = hardwareMap.get(DistanceSensor.class, "right");

        // Loop while the Op Mode is running
        waitForStart();
        while (opModeIsActive()) {
            // If the distance in centimeters is less than 10, set the power to 0.3
           telemetry.addData("Left:", left.getDistance(DistanceUnit.CM));
           telemetry.addData("Right:", right.getDistance(DistanceUnit.CM));
           telemetry.update();

        }
    }
}
