package pedroPathing.examples;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp (name = "AAA Chassis Teleop", group = "Examples")
public class AAA_Chassis_Teleop extends OpMode {
    public DcMotor frontLeftMotor;
    public DcMotor backLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backRightMotor;
    public double speed;

    @Override
    public void init(){
        frontLeftMotor = hardwareMap.dcMotor.get("leftFront");
        backLeftMotor = hardwareMap.dcMotor.get("leftRear");
        frontRightMotor = hardwareMap.dcMotor.get("rightFront");
        backRightMotor = hardwareMap.dcMotor.get("rightRear");
        speed = 1.0;

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void loop(){
        double x = -gamepad1.left_stick_x ; //left and right
        double y = gamepad1.left_stick_y;  //forward and backwards
        double rx = speed * gamepad1.right_stick_x; //spins the robot

        telemetry.addData("gamepad1.right_stick_x: ", gamepad1.right_stick_x );
        telemetry.update();
        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
        double frontLeftPower = (y + x - rx) / denominator;
        double backLeftPower = (y - x - rx) / denominator;
        double frontRightPower = (y - x + rx) / denominator;
        double backRightPower = (y + x + rx) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        telemetry.addData("frontLeftMotor: ", frontLeftPower);
        telemetry.update();

        backLeftMotor.setPower(backLeftPower);
        telemetry.addData("backLeftMotor: ", backLeftPower);
        telemetry.update();

        frontRightMotor.setPower(frontRightPower);
        telemetry.addData("frontRightMotor: ", frontRightPower);
        telemetry.update();

        backRightMotor.setPower(backRightPower);
        telemetry.addData("backRightMotor: ", backRightPower);
        telemetry.update();
    }

    @Override
    public void stop(){
        telemetry.addData("StopRequested: ", 0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }
}

