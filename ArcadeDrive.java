package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="ArcadeDrive", group="Linear Opmode")
//@Disabled

public class ArcadeDrive extends LinearOpMode{
    // Declare OpMode members.
    DcMotor leftMotor, rightMotor;
    float   leftPower, rightPower, xValue, yValue;

    // called when init button is  pressed.
    @Override
    public void runOpMode() throws InterruptedException
    {
        leftMotor = hardwareMap.dcMotor.get("Left Motor");
        rightMotor = hardwareMap.dcMotor.get("Right Motor");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        while (opModeIsActive())
        {
            yValue = gamepad1.left_stick_y;
            xValue = gamepad1.right_stick_x;

            rightPower =  yValue - xValue;
            leftPower = yValue + xValue;

            leftMotor.setPower(Range.clip(leftPower, -1.0, 1.0));
            rightMotor.setPower(Range.clip(rightPower, -1.0, 1.0));

            telemetry.addData("Mode", "running");
            telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
            telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);
            telemetry.update();

            idle();
        }
    }
}
