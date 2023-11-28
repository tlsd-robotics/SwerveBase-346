package Common;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/** 
 * <p>Creates a ThrustMaster specific {@link Joystick} including methods for every button and axis</p>
 */
public class ThrustMaster  {

    Joystick joystick;

    JoystickButton trigger;
    JoystickButton middle;
    JoystickButton left;
    JoystickButton right;
    JoystickButton leftFrontLeft;
    JoystickButton leftFrontMiddle;
    JoystickButton leftFrontRight;
    JoystickButton leftBackRight;
    JoystickButton leftBackMiddle;
    JoystickButton leftBackLeft; 
    JoystickButton rightFrontRight;
    JoystickButton rightFrontMiddle;
    JoystickButton rightFrontLeft;
    JoystickButton rightBackLeft;
    JoystickButton rightBackMiddle;
    JoystickButton rightBackRight;

//==============================================================================
//=============================== Joystick IDs ===================================
    private final int TRIGGER = 1;
    private final int MIDDLE = 2;
    private final int LEFT = 3;
    private final int RIGHT = 4;

    private final int LEFT_FRONT_LEFT = 5;
    private final int LEFT_FRONT_MIDDLE = 6;
    private final int LEFT_FRONT_RIGHT = 7;
    private final int LEFT_BACK_RIGHT = 8;
    private final int LEFT_BACK_MIDDLE = 9;
    private final int LEFT_BACK_LEFT = 10;

    private final int RIGHT_FRONT_RIGHT = 11;
    private final int RIGHT_FRONT_MIDDLE = 12;
    private final int RIGHT_FRONT_LEFT = 13;
    private final int RIGHT_BACK_LEFT = 14;
    private final int RIGHT_BACK_MIDDLE = 15;
    private final int RIGHT_BACK_RIGHT = 16;

    private final int xAxis = 1;
    private final int yAxis = 2;
    private final int zAxis = 3;
    private final int throttleAxis = 4;
    private final int hatXAxis = 5;
    private final int hatYAxis = 6;


//==============================================================================
//============================== Constructor ===================================

    public ThrustMaster(int port) {
        this.joystick = new Joystick(port);

        configureButtons(this.joystick);
    }


//==============================================================================
//============================ Create Buttons ==================================
    private void configureButtons(Joystick joy) {
        this.trigger = new JoystickButton(joy, TRIGGER);
        this.middle = new JoystickButton(joy, MIDDLE);
        this.left = new JoystickButton(joy, LEFT);
        this.right = new JoystickButton(joy, RIGHT);
        this.leftFrontLeft = new JoystickButton(joy, LEFT_FRONT_LEFT);
        this.leftFrontMiddle = new JoystickButton(joy, LEFT_FRONT_MIDDLE);
        this.leftFrontRight = new JoystickButton(joy, LEFT_FRONT_RIGHT);
        this.leftBackRight = new JoystickButton(joy, LEFT_BACK_RIGHT);
        this.leftBackMiddle = new JoystickButton(joy, LEFT_BACK_MIDDLE);
        this.leftBackLeft = new JoystickButton(joy, LEFT_BACK_LEFT);
        this.rightFrontRight = new JoystickButton(joy, RIGHT_FRONT_RIGHT);
        this.rightFrontMiddle = new JoystickButton(joy, RIGHT_FRONT_MIDDLE);
        this.rightFrontLeft = new JoystickButton(joy, RIGHT_FRONT_LEFT);
        this.rightBackLeft = new JoystickButton(joy, RIGHT_BACK_LEFT);
        this.rightBackMiddle = new JoystickButton(joy, RIGHT_BACK_MIDDLE);
        this.rightBackRight = new JoystickButton(joy, RIGHT_BACK_RIGHT);

    }

//==============================================================================
//================================ Getters =====================================

    /**
     * @return Joystick Object
     */
    public Joystick getJoy() {
        return joystick;
    }

    /**
     * @return Joystick's Trigger button
     */    
    public JoystickButton getTrigger() {
        return trigger;
    }

    /**
     * @return Middle/bottom button on the top of the joystick
     */    
    public JoystickButton getMiddle() {
        return middle;
    }

    /**
     * @return left button on top of joystick
     */    
    public JoystickButton getLeft() {
        return left;
    }

    /**
     * @return Right button on top of joystick
     */    
    public JoystickButton getRight() {
        return right;
    }

    /**
     * @return Return the front left button on the left side of the joystick.
     */    
    public JoystickButton getLeftFrontLeft() {
        return leftFrontLeft;
    }

    /**
     * @return Return the front middle button on the left side of the joystick.
     */    
    public JoystickButton getLeftFrontMiddle() {
        return leftFrontMiddle;
    }

    /**
     * @return Return the front right button on the left side of the joystick.
     */    
    public JoystickButton getLeftFrontRight() {
        return leftFrontRight;
    }

    /**
     * @return Return the back right button on the left side of the joystick.
     */    
    public JoystickButton getLeftBackRight() {
        return leftBackRight;
    }

    /**
     * @return Return the back middle button on the left side of the joystick.
     */    
    public JoystickButton getLeftBackMiddle() {
        return leftBackMiddle;
    }

    /**
     * @return Return the back left button on the left side of the joystick.
     */    
    public JoystickButton getLeftBackLeft() {
        return leftBackLeft;
    }

    /**
     * @return Return the front right button on the right side of the joystick.
     */    
    public JoystickButton getRightFrontRight() {
        return rightFrontRight;
    }

    /**
     * @return Return the front middle button on the right side of the joystick.
     */    
    public JoystickButton getRightFrontMiddle() {
        return rightFrontMiddle;
    }

    /**
     * @return Return the front left button on the right side of the joystick.
     */    
    public JoystickButton getRightFrontLeft() {
        return rightFrontLeft;
    }

    /**
     * @return Return the back left button on the right side of the joystick.
     */    
    public JoystickButton getRightBackLeft() {
        return rightBackLeft;
    }

    /**
     * @return Return the back middle button on the right side of the joystick.
     */    
    public JoystickButton getRightBackMiddle() {
        return rightBackMiddle;
    }

    /**
     * @return Return the back right button on the right side of the joystick.
     */    
    public JoystickButton getRightBackRight() {
        return rightBackRight;
    } 
    
    /**
     * @return Get joystick's X (horizontal) raw axis value
     */    
    public double getX() {
        return joystick.getRawAxis(xAxis);
    }

    /**
     * @return Get joystick's Y (vertical) raw axis value
     */    
    public double getY() {
        return joystick.getRawAxis(yAxis);
    }

    /**
     * @return Get joystick's Z (twist) raw axis value
     */    
    public double getZ() {
        return joystick.getRawAxis(zAxis);
    }

    /**
     * @return Get joystick's throttle value mapped to a range of <code>0 -> 1</code>
     */    
    public double getThrottle() {
        return Utilities.map(joystick.getRawAxis(throttleAxis), 1, -1, 0, 1);
    }

    /**
     * Note: Hat or sometimes called POV, is the clicky thumbstick on the top of the joystick. These axis values can be used like a gamepad's dpad
     * @return Get joystick HAT X value. 
     */    
    public double getHatX() {
        return joystick.getRawAxis(hatXAxis);
    }

    /**
     * Note: Hat or sometimes called POV, is the clicky thumbstick on the top of the joystick. These axis values can be used like a gamepad's dpad
     * @return Get joystick HAT Y value. 
     */  
    public double getHatY() {
        return joystick.getRawAxis(hatYAxis);
    }

}
