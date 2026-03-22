package frc.robot;

public class Constants 
{

    /**
     * the constant gravity on earth, in feet per second squared
     */
    public static double GRAVITY = 32.174;

    /**
     * The conversion ratio of feet to meters
     */
    public static double FEET_PER_METER = 3.2804;

    public static double DEGREES_PER_RADIANS = 57.295779513;

    public static double TURRETX = 0;
    public static double TURRETY = 0;

    public static double SHOOTERHEIGHT = 2.208;

    public static double[] shot_REDHUB   = {39.047, 13.193,  6};
    public static double[] shot_LEFTRED  = {46.612, 19.79,   0};
    public static double[] shot_RIGHTRED = {46.612,  6.5967, 0};

    public static double[] shot_BLUEHUB   = {15.13, 13.193,  6};
    public static double[] shot_LEFTBLUE  = {7.565, 19.79,   0};
    public static double[] shot_RIGHTBLUE = {7.565,  6.5967, 0};

    public static double[] shot_LEFTNEUTRAL  = {27.088, 19.79,   0};
    public static double[] shot_RIGHTNEUTRAL = {27.088,  6.5967, 0};

    /**
     * the current "mode" of the robot (auto or teleop)
     */
    public static enum eMode
    {
        AUTO, 
        TELEOP;
    }

    /**
     * enum that stores all the states for the aiming state machine
     */
    public static enum eAim
    {
        ZERO,
        TRANSITION,
        FIRE;
    }

    /**
     * enum that stores all the states for the autonomous state machine
     */
    public static enum eAuto
    {
        REST,
        TODEPOT,
        COLLECTDEPOT,
        TOOUTPOST,
        COLLECTOUTPOST,
        TOCENTER
    }


    /**
     * enum that stores all the starting positions for the robot
     */
    public static enum eInitPose
    {
        LEFT,
        MIDDLE,
        RIGHT
    }

    /**
     * enum that stores all the different goals or options during autonomous
     */
    public static enum eAutoGoal
    {
        NOMOVE,
        DEPOT,
        OUTPOST,
        DEPOT_THEN_OUTPOST,
        OUTPOST_THEN_DEPOT,
    }

    /*   JOYSTICK INPUTS 
     * 1 - start shooter (trigger finger)
     * 2 - stop shooter  (back thumb)
     * 3 - intake        (left thumb)
     * 4 - reset gyro    (right thumb)
     * 7 - brake         ()
    */

    /*   CONTROLLER INPUTS 
     * right bumper - shoot
     * left bumper  - stop
     * x            - intake
     * a            - brake
     * y            - reset gyro
    */
    

}