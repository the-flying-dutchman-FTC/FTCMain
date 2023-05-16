package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode drives a 4-motor Omni-Directional robot using two joysticks.
 *
 * Controls:
 * 1) Left joystick moves the robot forward, backward, and sideways.
 * 2) Right joystick spins the robot.
 *
 */
@TeleOp
public class OmniDriveOpMode_Linear extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor MotorZero = null;
    private DcMotor MotorOne = null;
    private DcMotor MotorTwo = null;
    private DcMotor MotorThree = null;

    @Override
    public void runOpMode() {
        // TODO: CHANGE MOTOR NAMES
        // Initialize the hardware variables.
        MotorZero = hardwareMap.get(DcMotor.class, "motor_zero");
        MotorOne = hardwareMap.get(DcMotor.class, "motor_one");
        MotorTwo = hardwareMap.get(DcMotor.class, "motor_two");
        MotorThree = hardwareMap.get(DcMotor.class, "motor_three");

        // TODO: test to see proper directions, ori was reverse reverse forward forward
        // Set the motor directions.
        MotorZero.setDirection(DcMotor.Direction.REVERSE);
        MotorOne.setDirection(DcMotor.Direction.FORWARD);
        MotorTwo.setDirection(DcMotor.Direction.REVERSE);
        MotorThree.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double axial = gamepad1.left_stick_y; // changed from -gamepad1.left_stick_y
            double lateral = -gamepad1.left_stick_x; // changed from gamepad1.left_stick_x
            double yaw = gamepad1.right_stick_x;

// To add strafing capability, control the lateral movement of the robot.
// Combine the joystick requests for each axis-motion to determine each wheel's power.

            double MotorZeroPower = -0.5 * (axial + lateral) + yaw; // Front Left
            double MotorOnePower = -0.5 * (axial - lateral) - yaw; // Front Right
            double MotorTwoPower = -0.5 * (axial - lateral) + yaw; // Back Left
            double MotorThreePower = -0.5 * (axial + lateral) - yaw; // Back Right



            // Send calculated power to wheels.
            MotorZero.setPower(MotorZeroPower);
            MotorTwo.setPower(MotorTwoPower);
            MotorOne.setPower(MotorOnePower);
            MotorThree.setPower(MotorThreePower);
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", MotorZeroPower, MotorOnePower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", MotorTwoPower, MotorThreePower);
            telemetry.update();
        }
    }
}

