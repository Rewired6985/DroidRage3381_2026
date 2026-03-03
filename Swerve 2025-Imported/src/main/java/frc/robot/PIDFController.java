package frc.robot;

import java.lang.Math;

public class PIDFController
{

    //TODO add docs, add functions for setting variables

    /**
     * Proportional gain
     */
    public double m_Kp;
    /**
     * Integral gain
     */
    public double m_Ki;
    /**
     * Derivative gain
     */
    public double m_Kd;
    /**
     * Minimum output that is intended in the system (e.g. a Falcon should never have an output that is less than -0.95, or it may redline)
     */
    public double m_LimitMin = -1;
    /**
     * Maximum output that is intended in the system (e.g. a Falcon should never have an output that is greater than 0.95, or it may redline)
     */
    public double m_LimitMax =  1;
    /**
     * internal variable; stores previous target for generating trajectories
     */
    private double m_LastTarget;

    /**
     * internal variable; used for storing position FFWD offsets
     */
    private double m_PositionOffset;

    /**
     * dev tool; used for troubleshooting 
     */
    public double m_PIDresult;
    /**
     * difference between target position and current position
     */
    public double m_Error = 0;
    /**
     * internal variable; used for calculating derivative
     */
    public double m_LastError = 0;

    public double m_Iterm = 0;
    public double m_Dterm = 0;

    public double m_PIDlasttime_s = 0;

    public double m_FFWDPeriod_ms;
    public double m_LastCoefficient;
    public double m_ReferenceTime_ms;
    public double m_FFWDOffset;
    public double[] m_FFWDResults = {0,0};

    /**
     * 
     * @param Kp
     * @param Ki
     * @param Kd
     * @param FFWDPeriod
     */
    public PIDFController(double Kp, double Ki, double Kd, double FFWDPeriod_ms)
    {
        m_Kp            = Kp;
        m_Ki            = Ki;
        m_Kd            = Kd;
        m_FFWDPeriod_ms = FFWDPeriod_ms;
        m_ReferenceTime_ms = -(FFWDPeriod_ms + 1);
    }

    /**
     * Calculates a PID value using the constants from the declaration of the PIDFController. 
     * @return sum of the Pterm, Iterm, Dterm
     * @param time_s (current system time, in seconds)
     */
    public double CalcPID(double time_s)
    {

        double deltaTime_s = (time_s - m_PIDlasttime_s);
        m_PIDlasttime_s = time_s;

        m_Iterm = m_Iterm + ((m_Error * deltaTime_s) * m_Ki);

        if      (m_Iterm < m_LimitMin) m_Iterm = m_LimitMin;
        else if (m_Iterm > m_LimitMax) m_Iterm = m_LimitMax;

        if (deltaTime_s > 0) m_Dterm = (((m_Error - m_LastError)/deltaTime_s) * m_Kd);
        else                 m_Dterm = 0;

        m_PIDresult = (m_Error * m_Kp) + m_Iterm + m_Dterm;

        m_LastError = m_Error;

        if      (m_PIDresult < m_LimitMin) m_PIDresult = m_LimitMin;
        else if (m_PIDresult > m_LimitMax) m_PIDresult = m_LimitMax;

        return m_PIDresult;
    }

    public double[] CalcFFWDVelocity(double coefficient, double target, double time_ms)
    {
        if (m_LastCoefficient != coefficient)
        {
            m_FFWDOffset     = m_LastCoefficient;
            m_PositionOffset = m_LastTarget;
            m_ReferenceTime_ms = time_ms;
        }

        if ((time_ms - m_ReferenceTime_ms) > m_FFWDPeriod_ms)
        {
            m_FFWDResults[0] = coefficient;
            m_FFWDResults[1] = target;
        }
        else
        {
            m_FFWDResults[0] = (-Math.cos((Math.PI * (time_ms - m_ReferenceTime_ms))/ m_FFWDPeriod_ms) + 1) *
                           ((coefficient - m_FFWDOffset)/2) +
                           m_FFWDOffset;

            m_FFWDResults[1] = (-Math.cos((Math.PI * (time_ms - m_ReferenceTime_ms))/ m_FFWDPeriod_ms) + 1) *
                    ((target - m_PositionOffset)/2) +
                    m_PositionOffset;
        }

        m_LastCoefficient = coefficient;
        m_LastTarget      = target;

        return m_FFWDResults;
    }

    public double[] CalcFFWDPosition(double coefficient, double target, double time_ms)
    {
        if (m_LastTarget != target)
        {
            m_PositionOffset = m_LastTarget;
            m_ReferenceTime_ms = time_ms;
        }

        if ((time_ms - m_ReferenceTime_ms) > m_FFWDPeriod_ms)
        {
            m_FFWDResults[0] = 0;
            m_FFWDResults[1] = target;
        }
        else
        {
            m_FFWDResults[0] = (Math.cos((Math.PI * (time_ms - m_ReferenceTime_ms))/ m_FFWDPeriod_ms) + 1) *
                    ((coefficient)/2);

            m_FFWDResults[1] = (-Math.cos((Math.PI * (time_ms - m_ReferenceTime_ms))/ m_FFWDPeriod_ms) + 1) *
                    ((target - m_PositionOffset)/2) +
                    m_PositionOffset;
        }

        m_LastTarget = target;

        return m_FFWDResults;
    }

}
