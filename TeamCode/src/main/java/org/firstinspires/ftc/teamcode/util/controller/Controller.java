package org.firstinspires.ftc.teamcode.util.controller;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.util.Pose2d;

/**
 * A Controller
 */
public class Controller {
    private final Gamepad gamepad;
    public final Button leftBumper;
    public final Button rightBumper;
    public final Button dpadUp;
    public final Button dpadDown;
    public final Button dpadLeft;
    public final Button dpadRight;
    public final Button x;
    public final Button a;
    public final Button b;
    public final Button y;
    public final Button leftStickButton;
    public final Button rightStickButton;
    public final Button options;
    public final Button touchpad;
    public final Button share;
    public final Button ps;
    public final Axis leftTrigger;
    public final Axis rightTrigger;
    public final Joystick leftStick;
    public final Joystick rightStick;

    /**
     * Instantiates the Controller
     *
     * @param gamepad the gamepad to use
     */
    public Controller(Gamepad gamepad) {
        this.gamepad = gamepad;

        leftBumper = new Button();
        rightBumper = new Button();
        dpadUp = new Button();
        dpadDown = new Button();
        dpadLeft = new Button();
        dpadRight = new Button();
        x = new Button();
        a = new Button();
        b = new Button();
        y = new Button();
        leftStickButton = new Button();
        rightStickButton = new Button();
        options = new Button();
        touchpad = new Button();
        share = new Button();
        ps = new Button();

        leftTrigger = new Axis();
        rightTrigger = new Axis();

        leftStick = new Joystick();
        rightStick = new Joystick();
    }

    public Pose2d getDrivePower(){ return new Pose2d(leftStick.x.value(), leftStick.y.value(), rightStick.x.value()); }

    public void buzz(){ gamepad.rumble(200); }
    public void buzz(int length){ gamepad.rumble(length); }

    public void green(){ gamepad.setLedColor(0, 1, 0, 1000); }
    public void red(){ gamepad.setLedColor(1, 0, 0, 1000); }
    public void blue(){ gamepad.setLedColor(0, 0, 1, 1000); }

    public boolean somethingPressed(){ return leftBumper.isPressed() || rightBumper.isPressed() || dpadUp.isPressed() ||
            dpadDown.isPressed() || a.isPressed() || b.isPressed() || x.isPressed() || leftTrigger.moved() || rightTrigger.moved(); }

    /**
     * Updates all Buttons, Axes, and Joysticks
     */
    public void update() {
        leftBumper.update(gamepad.left_bumper);
        rightBumper.update(gamepad.right_bumper);
        dpadUp.update(gamepad.dpad_up);
        dpadDown.update(gamepad.dpad_down);
        dpadLeft.update(gamepad.dpad_left);
        dpadRight.update(gamepad.dpad_right);
        a.update(gamepad.a);
        b.update(gamepad.b);
        x.update(gamepad.x);
        y.update(gamepad.y);
        leftStickButton.update(gamepad.left_stick_button);
        rightStickButton.update(gamepad.right_stick_button);
        options.update(gamepad.options);
        touchpad.update(gamepad.touchpad);
        share.update(gamepad.share);
        ps.update(gamepad.ps);

        leftTrigger.update(gamepad.left_trigger);
        rightTrigger.update(gamepad.right_trigger);

        leftStick.update(gamepad.left_stick_x, -gamepad.left_stick_y);
        rightStick.update(gamepad.right_stick_x, -gamepad.right_stick_y);
    }
}
