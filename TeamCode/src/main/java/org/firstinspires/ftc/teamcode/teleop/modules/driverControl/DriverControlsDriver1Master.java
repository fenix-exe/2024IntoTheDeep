package org.firstinspires.ftc.teamcode.teleop.modules.driverControl;

import com.qualcomm.robotcore.hardware.Gamepad;

public class DriverControlsDriver1Master extends DriverControls{
    public DriverControlsDriver1Master(Gamepad gamepad1, Gamepad gamepad2, double y) {
        super(gamepad1, gamepad2, y);
    }

    @Override
    public boolean depositBack() {
        return super.depositBack() || (super.gamepad1current.left_trigger > 0.5 && !(super.gamepad1previous.left_trigger > 0.5));
    }

    @Override
    public boolean specimenSampleIntake() {
        return super.specimenSampleIntake() || super.gamepad1current.right_trigger > 0.5;
    }

    @Override
    public double slideMovement() {
        if (Math.abs(gamepad2current.left_stick_y) > 0.65){
            return -gamepad2current.left_stick_y;
        }
        /*if (Math.abs(gamepad1current.right_stick_y) > 0.65){
            return -gamepad1current.right_stick_y;
        }*/
        return 0;
    }
    @Override
    public boolean slideStopped(){
        if (Math.abs(gamepad2previous.left_stick_y) > 0.5 && !(Math.abs(gamepad2current.left_stick_y) > 0.5)){
            return true;
        }
        return Math.abs(gamepad1previous.right_stick_y) > 0.65 && !(Math.abs(gamepad1current.right_stick_y) > 0.3);
    }
    @Override
    public boolean pickupAndDepositSpecimens() {
        return super.pickupAndDepositSpecimens() || (gamepad1current.right_bumper && !gamepad1previous.right_bumper);
    }

    @Override
    public boolean slowMode() {
        return gamepad1current.left_bumper;
    }
}
