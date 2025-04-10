package org.firstinspires.ftc.teamcode.teleop.modules.driverControl;

public interface DriveControlMap {
    double forwardDrive();
    double strafeDrive();
    double turnDrive();
    boolean driveTypeSwitch();
    boolean resetIMU();
    boolean slowMode();
    double pivotJoystick();
    double slideMovement();
    boolean slideStopped();
    boolean linearActuatorUp();
    boolean linearActuatorDown();
    boolean specimenSampleIntake();
    boolean outtake();
    boolean intake();
    boolean grabSampleFromOutside();
    boolean enterIntakePosition();
    boolean pickupAndDepositSpecimens();
    boolean depositBack();
    boolean escapePresets();
    boolean removeArmRules();
    boolean diffUp();
    boolean diffDown();
    boolean hang();
    boolean turnOffAutoGrab();
    boolean programPos();



}
