package org.firstinspires.ftc.teamcode.teleop.modules.driverControl;

public interface DriveControlMap {
    //gamepad 1 controls
    boolean slowMode();
    //makes robot move slower, is a toggle
    boolean driveTypeSwitch();
    //switches robot modes, is a toggle
    boolean resetIMU();
    boolean emergencyStop();
    //turns off robot completely
    boolean undoEmergencyStop();
    //turns robot back on
    //gamepad 2 controls
    boolean slidesFullyUp();
    //slides go to max
    boolean slidesFullyDown();
    //slides go to min
    boolean pivotParallel();
    //pivot parallel to ground
    boolean pivotPerp();
    //pivot perpendicular to ground
    double pivotJoystick();
    double slideMovement();
    //joystick control of slides
    double degreeOfFreedomX();
    //controls x degree of freedom
    double degreeOfFreedomY();
    //controls y degree of freedom
    boolean intakePower();
    //turns intake on/off, toggle
    boolean intakeDirection();
    //changes intake direction, toggle
    float intakenewForward();
    //instead of toggle, hold
    float intakenewBackward();
    //instead of toggle, hold

    boolean resetWrist();
    boolean submersibleIntakeReady();
    boolean acsent1Park();
    boolean drivingPos();
    boolean depositReadyBackTopBucket();
    boolean depositReadyFrontTopBucket();
    boolean depositReadyBackBottomBucket();
    boolean depositReadyFrontBottomBucket();
    boolean setNewDrivingPos();
    boolean setNewDepositReadyBackTopBucket();
    boolean setNewDepositReadyFrontTopBucket();
    boolean setNewDepositReadyBackBottomBucket();
    boolean setNewDepositReadyFrontBottomBucket();
    boolean escapePresets();



}
