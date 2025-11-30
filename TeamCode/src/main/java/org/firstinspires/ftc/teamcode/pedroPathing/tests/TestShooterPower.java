package org.firstinspires.ftc.teamcode.pedroPathing.tests;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Autonomous(name = "Shooter Power Test", group = "Vision")
public class TestShooterPower extends LinearOpMode {

    private DcMotorEx shooterMotor;
    private Limelight3A limelight;

    @Override
    public void runOpMode() throws InterruptedException {
        // Hardware initialization
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        shooterMotor = hardwareMap.get(DcMotorEx.class, "shooterMotor");

        telemetry.setMsTransmissionInterval(11);

        // Start Limelight early
        limelight.start();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.clear();

            // Switch to pipeline 0 (your AprilTag 21 pipeline)
            limelight.pipelineSwitch(0);
            sleep(150);  // Limelight needs ~120-150ms to stabilize

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                double Ta = result.getTa();  // Tag area
                telemetry.addData("Pipeline 0", "Detection");
                telemetry.addData("Ta", Ta);

                // ---- SAFE MOTOR POWER FORMULA ----
                // Prevent Ta = 0 (divide-by-zero crash)
                double safeTa = Math.max(Ta, 0.01);

                // Inverse relationship (closer tag = more power)
                double motorPower = 1.0 / safeTa;

                // Limit power into valid FTC motor range
                motorPower = Math.min(motorPower, 1.0);

                shooterMotor.setPower(-motorPower);

                telemetry.addData("Motor Power", motorPower);

            } else {
                telemetry.addData("Pipeline 0", "NO TAG FOUND");
                shooterMotor.setPower(0);
            }

            telemetry.update();
            sleep(50);
        }

        // Stop Limelight safely
        limelight.stop();
    }
}
