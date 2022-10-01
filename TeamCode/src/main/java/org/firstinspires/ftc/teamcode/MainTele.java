package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class MainTele extends LinearOpMode {
    
    double intergalSum = 0;
    double KP = .01;
    double KI = 0;
    double KD = 0;
    ElapsedTime timer = new ElapsedTime();
    private double lastError;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor leftfront;
        DcMotor leftback;
        DcMotor rightfront;
        DcMotor rightback;
        DcMotor fourbar;
        CRServo rightintakeservo;
        CRServo leftintakeservo;

         int ClawState = 0;
         boolean ClawStateTripper = false;
        int FourbarState = 0;
        boolean FourbarStateTripper = false;



        leftfront = hardwareMap.get(DcMotor.class, "leftfront");
        leftback = hardwareMap.get(DcMotor.class, "leftback");
        rightfront = hardwareMap.get(DcMotor.class, "rightfront");
        rightback = hardwareMap.get(DcMotor.class, "rightback");
        fourbar = hardwareMap.get(DcMotor.class, "fourbar");
        rightintakeservo = hardwareMap.get(CRServo.class, "rightintakeservo");
        leftintakeservo = hardwareMap.get(CRServo.class, "leftintakeservo");

        leftfront.setDirection(DcMotor.Direction.REVERSE);
        rightfront.setDirection(DcMotor.Direction.FORWARD);
        leftback.setDirection(DcMotor.Direction.REVERSE);
        rightback.setDirection(DcMotor.Direction.FORWARD);
        leftintakeservo.setDirection(CRServo.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {
            int Speed = 1;
            int DIRECTION = 1;
            double Drive = -gamepad1.left_stick_y * DIRECTION;
            double Strafe = gamepad1.left_stick_x * DIRECTION;
            double Turn = gamepad1.right_stick_x * DIRECTION;


            leftfront.setPower((Drive + Strafe + Turn) * Speed);
            leftback.setPower((Drive - Strafe + Turn) * Speed);
            rightfront.setPower((Drive - Strafe - Turn) * Speed);
            rightback.setPower((Drive + Strafe - Turn) * Speed);

            if (gamepad1.right_trigger >= .1) {
                leftintakeservo.setPower(.7);
                rightintakeservo.setPower(.7);
            } else if (gamepad1.left_trigger >= .1) {
                leftintakeservo.setPower(-.7);
                rightintakeservo.setPower(-.7);
            } else {
                leftintakeservo.setPower(0);
                rightintakeservo.setPower(0);
            }

            if (gamepad1.right_bumper) {
                fourbar.setPower(-.65);
            } else if (gamepad1.left_bumper) {
                fourbar.setPower(.5);
            } else {
                fourbar.setPower(0);
            }


            // below is the code for our 3 week robot
//            if (gamepad1.left_bumper) {
//                if (!FourbarStateTripper) {
//                    FourbarStateTripper = true;
//                    FourbarState++;
//                    if (FourbarState == 3) {
//                        FourbarState = 0;
//                    }
//                }
//            } else {
//                FourbarStateTripper = false;
//            }
//            switch (FourbarState) {
//                case 1:
//                    fourbar.setPosition(.92);
//                    break;
//                case 2:
//                    fourbar.setPosition(.87);
//                    break;
//                case 3:
//                    fourbar.setPosition(.8);
//                    sleep(700);
//                    fourbar.setPosition(.9);
//                    sleep(999999999);
//                    break;
//                default:
//                    fourbar.setPosition(1);
//                    break;
//            }

//            if (gamepad1.right_bumper) {
//                if (!ClawStateTripper) {
//                    ClawStateTripper = true;
//                    ClawState++;
//                    if (ClawState == 2) {
//                        ClawState = 0;
//                    }
//                }
//            } else {
//                ClawStateTripper = false;
//            }
//            switch (ClawState) {
//                case 1:
//                    claw.setPosition(.8);
//                    break;
//                default:
//                    claw.setPosition(.65);
//                    break;
//            }

        }
    }
    public double PIDController(double reference, double state) {
        double error = reference - state;
        intergalSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * KP) + (derivative * KD) + (intergalSum * KI);
        return output;
    }
}
