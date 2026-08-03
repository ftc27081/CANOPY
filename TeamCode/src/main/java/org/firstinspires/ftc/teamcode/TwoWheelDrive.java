package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@TeleOp(name="Basic Drive")
public class TwoWheelDrive extends LinearOpMode {


   private DcMotor frontRight;
   private DcMotor frontLeft;

   public void runOpMode(){
       initialization();
       telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
       telemetry.addData("Status", "Waiting");
       telemetry.update();
       waitForStart();

       while (opModeIsActive()) {
           this.drive();
       }

   }



    public void initialization() {
        // Initialization
        frontLeft = hardwareMap.get(DcMotorEx.class, "fl2  nbMotor");
        frontRight = hardwareMap.get(DcMotorEx.class, "frMotor");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void drive(){

        double turn = gamepad1.right_stick_x;

        double leftPower  = -gamepad1.left_stick_y + turn;
        double rightPower = -gamepad1.left_stick_y - turn;

        double maxPower = Math.max(Math.abs(leftPower), Math.abs(rightPower));

        if (maxPower > 1.0) {
            leftPower  /= maxPower;
            rightPower /= maxPower;
        }

        frontLeft.setPower(leftPower);
        frontRight.setPower(rightPower);

    }




}
