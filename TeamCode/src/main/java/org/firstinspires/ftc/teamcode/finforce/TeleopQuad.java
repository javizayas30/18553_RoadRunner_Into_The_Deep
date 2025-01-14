package org.firstinspires.ftc.teamcode.finforce;
/*
Starter Code for Quad Training Robot 2024
Modified by michaudc 2017, 2023, 2024
*/
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;
@TeleOp(name="MaristBot2024: Quad Training 2024", group="Training")
//@Disabled
public class TeleopQuad extends OpMode {
    // Create instance of MaristBaseRobot2024
    MaristBaseRobot2024_Quad robot  = new MaristBaseRobot2024_Quad();

    int armPos = 0; //sets arm default position to 0
    int sliderPos = 0;

    double ARM_SPEED = 0.8;
    double SLIDER_SPEED = 0.8;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Robot Ready");    //
        // Set to Run without Encoder for Tele Operated
        robot.leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //For Arm Hold Code
        robot.leftArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armPos = robot.leftArm.getCurrentPosition();

        robot.rightArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        sliderPos = robot.rightArm.getCurrentPosition();
    }
    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }
    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double leftFrontPower = gamepad1.left_stick_y-gamepad1.left_stick_x-gamepad1.right_stick_x;
        double leftRearPower = gamepad1.left_stick_y+gamepad1.left_stick_x-gamepad1.right_stick_x;
        double rightFrontPower = gamepad1.left_stick_y+gamepad1.left_stick_x+gamepad1.right_stick_x;
        double rightRearPower = gamepad1.left_stick_y-gamepad1.left_stick_x+gamepad1.right_stick_x;
        robot.leftFront.setPower(leftFrontPower);
        robot.leftRear.setPower(leftRearPower);
        robot.rightFront.setPower(rightFrontPower);
        robot.rightRear.setPower(rightRearPower);

        // Left Arm Code
        double deltaArmPos = gamepad2.left_trigger-gamepad2.right_trigger;

        if (Math.abs(deltaArmPos) > 0.1) { // Arm is moving
            robot.leftArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.leftArm.setPower(deltaArmPos);
        }
        else { // Arm is holding Position
            armPos = robot.leftArm.getCurrentPosition();
            robot.leftArm.setTargetPosition(armPos);
            robot.leftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.leftArm.setPower(ARM_SPEED);
        }

        // Right Arm (Slider) Code
        if (gamepad1.y) {// Arm Up
            robot.rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightArm.setPower(0.8);
        }
        else if (gamepad1.a) { // Arm Down
            robot.rightArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            robot.rightArm.setPower(-0.8);
        }
        else{
            sliderPos=robot.rightArm.getCurrentPosition();
            robot.rightArm.setTargetPosition(sliderPos);
            robot.rightArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.rightArm.setPower(SLIDER_SPEED);
        }

        // Servo Control
        if (gamepad2.left_bumper) {
            robot.leftHand.setPosition(0.6); // Open
        }
        if (gamepad2.right_bumper) {
            robot.leftHand.setPosition(0.1); // Close
        }

    }
    /*
     * Code to run ONCE after the driver hits STOP
     */

    {
    }
}
