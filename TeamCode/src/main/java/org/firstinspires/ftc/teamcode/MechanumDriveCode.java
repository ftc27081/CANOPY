package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.LaunchZoneRed.TICKS_PER_REV;
import static java.util.Collections.max;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name="Mechanum DriveCode")
public class MechanumDriveCode extends LinearOpMode {


    double lastError = 0;
    double integralSum = 0;
    ElapsedTime timer = new ElapsedTime();
    public int runMotor = 0;
    public static double p = 0.012;
    public static double i = 0;
    public static double d = 0.00;
    public static double f = 0.00064;
    public static double targetValue1 = 1700;

    public static double targetValue = 1210;


    private DcMotorEx flMotor, frMotor, blMotor, brMotor, outakeMotor;
    static final double WHEEL_DIAMETER_INCHES = 1.504; // adjust for your wheels
    static final double COUNTS_PER_INCH = TICKS_PER_REV / (WHEEL_DIAMETER_INCHES * Math.PI);
    private DcMotor slideMotor1,slideMotor2;
    private Servo artifactGate,rampControl;

    public void drive() {
        //get joystick values
        double yPower = 0.7 * gamepad1.left_stick_y;   // forward and back
        double yaw = 0.7 * gamepad1.right_stick_x;  // turning
        double xPower = 0.7 * gamepad1.left_stick_x;   // drive

        //calculate powersf
        double frPower = yPower + xPower + yaw;
        double flPower = yPower - xPower - yaw;
        double brPower = yPower - xPower + yaw;
        double blPower = yPower + xPower - yaw;

        double maxPower = Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)), Math.max(Math.abs(blPower), Math.abs(brPower)));
        if (maxPower > 1.0) {
            flPower /= maxPower;
            blPower /= maxPower;
            frPower /= maxPower;
            brPower /= maxPower;
        }

        flMotor.setPower(flPower);
        frMotor.setPower(frPower);
        blMotor.setPower(blPower);
        brMotor.setPower(brPower);


    }


    public void wheelVelocity(DcMotorEx motor, double targetVelocity) {

        double currentVelocity = motor.getVelocity();
        double error = targetVelocity - currentVelocity;

        double dt = timer.seconds();
        timer.reset();

        telemetry.addData("Seconds passed", dt);
        telemetry.addData("milisceonds passed",dt*0.0001);
        telemetry.update();

        // Protect against divide-by-zero
        if (dt <= 0) return;

        // ----- FEEDFORWARD -----
        double fComponent = f * targetVelocity;

        // ----- PROPORTIONAL -----
        double pComponent = p * error;

        // ----- INTEGRAL -----
        integralSum += error * dt;
        double iComponent = i * integralSum;

        // ----- DERIVATIVE -----
        double derivative = (error - lastError) / dt;
        double dComponent = d * derivative;

        // ----- TOTAL POWER -----
        double power = fComponent + pComponent + iComponent + dComponent;

        // Clamp motor power
        power = Math.max(-1.0, Math.min(1.0, power));
        motor.setPower(power);

        lastError = error;

        // Telemetry
        telemetry.addData("Target", targetVelocity);
        telemetry.addData("Velocity", currentVelocity);
        telemetry.addData("Error", error);
        telemetry.addData("P", pComponent);
        telemetry.addData("I", iComponent);
        telemetry.addData("D", dComponent);
        telemetry.addData("F", fComponent);
    }



    public void wheelVelocityfar(DcMotorEx motor, double targetVelocity) {

        double integralSum1 = 0;
        double f1= 0.00043;
        double p1=0.00063;
        double i1=0.0000000000014;
        double d1= 0.00000041;
        double targetVelocity1= 1700;
        double currentVelocity1 = motor.getVelocity();
        double error1 = targetVelocity1 - currentVelocity1;

        double dt1 = timer.seconds();
        timer.reset();

        telemetry.addData("Seconds passed", dt1);
        telemetry.addData("milisceonds passed",dt1*0.0001);
        telemetry.update();

        // Protect against divide-by-zero
        if (dt1 <= 0) return;

        // ----- FEEDFORWARD -----
        double fComponent1 = f1 * targetVelocity1;

        // ----- PROPORTIONAL -----
        double pComponent1 = p1 * error1;

        // ----- INTEGRAL -----
        integralSum1 += error1 * dt1;
        double iComponent = i1 * integralSum1;

        // ----- DERIVATIVE -----
        double derivative = (error1 - lastError) / dt1;
        double dComponent = d * derivative;

        // ----- TOTAL POWER -----
        double power = fComponent1 + pComponent1+ iComponent + dComponent;

        // Clamp motor power
        power = Math.max(-1.0, Math.min(1.0, power));
        motor.setPower(power);

        lastError = error1;

        // Telemetry
        telemetry.addData("Target", targetVelocity);
        telemetry.addData("Velocity", currentVelocity1);
        telemetry.addData("Error", error1);
        telemetry.addData("P", pComponent1);
        telemetry.addData("I", iComponent);
        telemetry.addData("D", dComponent);
        telemetry.addData("F", fComponent1);
    }



    public void powerControl() {
        if (gamepad2.a) {
           runMotor = 1;
        }
        if (gamepad2.y){
            runMotor=2;
        }

        if (gamepad2.x){
            outakeMotor.setPower(-.01);
            sleep(2000);
            outakeMotor.setPower(0);
            runMotor = 0;
        }
    }

    public void release() {

        if (gamepad2.right_bumper) {
            artifactGate.setPosition(0.5);
        }
        if(gamepad2.left_bumper) {
            artifactGate.setPosition(1.0);
        }
    }



    public void logData() {
        if(gamepad1.b) {
            telemetry.addData("left motor:", slideMotor1.getCurrentPosition());
            telemetry.addData("right motor:", slideMotor2.getCurrentPosition());
        }
    }




    public void initialization() {
        // Initialization
        flMotor = hardwareMap.get(DcMotorEx.class, "flMotor");
        frMotor = hardwareMap.get(DcMotorEx.class, "frMotor");
        blMotor = hardwareMap.get(DcMotorEx.class, "blMotor");
        brMotor = hardwareMap.get(DcMotorEx.class, "brMotor");
        outakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        rampControl = hardwareMap.get(Servo.class,"rampControl");


        artifactGate = hardwareMap.get(Servo.class,"artifactGate");
        artifactGate.setDirection(Servo.Direction.FORWARD);
        artifactGate.setPosition(1.0);

        frMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        brMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        outakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        flMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        frMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        blMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        brMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);


        outakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry = new MultipleTelemetry(telemetry,FtcDashboard.getInstance().getTelemetry());


    }

    public void runOpMode() {
        // init
        initialization();

        // create multiple telemetries and add to dashboard
        telemetry.addData("Status", "Waiting");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            this.drive();
            this.powerControl();
            this.release();
            this.logData();
            if(runMotor==1) {
                this.wheelVelocity(outakeMotor,targetValue);
            }
            if (runMotor==2){

                this.wheelVelocityfar(outakeMotor,targetValue1);
            }

            telemetry.update();
        }
    }
}