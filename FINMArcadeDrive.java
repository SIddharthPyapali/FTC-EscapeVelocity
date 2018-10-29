/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 * <p>
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name = "FINAL TELE-OP", group = "Linear Opmode")
//@Disabled
public class FINMArcadeDrive extends LinearOpMode {

    DcMotor leftMotor, rightMotor, linearMotor, armMotor, intakeMotor, latchMotor;
    float leftPower, rightPower, xValue, yValue;
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        linearMotor = hardwareMap.dcMotor.get("Linear Motor");
        armMotor = hardwareMap.dcMotor.get("Arm Motor");
        intakeMotor = hardwareMap.dcMotor.get("Intake Motor");
        latchMotor = hardwareMap.dcMotor.get("Latch Motor");
        leftMotor = hardwareMap.dcMotor.get("Left Motor");
        rightMotor = hardwareMap.dcMotor.get("Right Motor");
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        linearMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            yValue = gamepad1.left_stick_y;
            xValue = gamepad1.right_stick_x;

            rightPower = yValue + xValue;
            leftPower = yValue - xValue;

            if (gamepad1.right_stick_button) {
                leftMotor.setPower(Range.clip(leftPower, -1.0, 1.0));
                rightMotor.setPower(Range.clip(rightPower, -1.0, 1.0));
            } else {
                leftMotor.setPower(Range.clip(leftPower, -.75, .75));
                rightMotor.setPower(Range.clip(rightPower, -.75, .75));
            }
            if (linearMotor.getCurrentPosition() >= 0) {
                linearMotor.setPower(Range.clip(gamepad2.left_stick_y, -1.0, 1.0));
            } else {
                linearMotor.setTargetPosition((-(0 - leftMotor.getCurrentPosition())) + 1);
            }
            latchMotor.setPower(Range.clip(gamepad2.right_stick_y, -1.0, 1.0));

            if (gamepad2.b){
                armMotor.setPower(.4);
                sleep(4000);
                armMotor.setPower(0);
            }
            if (gamepad2.x){
                armMotor.setPower(-.2);
                sleep (1000);
                armMotor.setPower(0);
            }

            if (gamepad2.right_bumper){
                intakeMotor.setPower(1.0);}
            if (gamepad2.left_bumper){
                intakeMotor.setPower(-1.0);}
            if (gamepad2.a){
                intakeMotor.setPower(0.0);}

            telemetry.addData("Mode", "running");
            telemetry.addData("stick", "  y=" + yValue + "  x=" + xValue);
            telemetry.addData("power", "  left=" + leftPower + "  right=" + rightPower);
            telemetry.addData("Intake Encoder", "Encoder=" + linearMotor.getCurrentPosition());
            telemetry.update();

            idle();
        }
    }
}