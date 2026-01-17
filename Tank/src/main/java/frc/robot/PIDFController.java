package frc.robot;

import java.lang.Math;

public class PIDFController
{

    public double m_Kp;
    public double m_Ki;
    public double m_Kd;
    public double m_LimitMin = -1;
    public double m_LimitMax =  1;
    public double m_LastTarget;
    public double m_PositionOffset;
    public double m_PIDresult;
    public double m_Error = 0;
    public double m_LastError = 0;

    public double m_Iterm = 0;
    public double m_Dterm = 0;

    public double m_PIDtime_s = 0;
    public double m_PIlasttime_s = 0;

    public double m_FFWDPeriod_ms;
    public double m_LastCoefficient;
    public double m_ReferenceTime_ms;
    public double m_FFWDOffset;
    public double[] m_FFWDResults = {0,0};

    public PIDFController(double Kp, double Ki, double Kd, double FFWDPeriod)
    {
        m_Kp            = Kp;
        m_Ki            = Ki;
        m_Kd            = Kd;
        m_FFWDPeriod_ms = FFWDPeriod;
        m_ReferenceTime_ms = -(FFWDPeriod + 1);
    }

    public double CalcPIDF()
    {

        double deltaTime_s = (m_PIDtime_s - m_PIlasttime_s);
        m_PIlasttime_s = m_PIDtime_s;

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

    public double[] CalcFFWD(double coefficient, double target, double time)
    {
        if (m_LastCoefficient != coefficient)
        {
            m_FFWDOffset     = m_LastCoefficient;
            m_PositionOffset = m_LastTarget;
            m_ReferenceTime_ms = time;
        }

        if ((time - m_ReferenceTime_ms) > m_FFWDPeriod_ms)
        {
            m_FFWDResults[0] = coefficient;
            m_FFWDResults[1] = target;
        }
        else
        {
            m_FFWDResults[0] = (-Math.cos((Math.PI * (time - m_ReferenceTime_ms))/ m_FFWDPeriod_ms) + 1) *
                           ((coefficient - m_FFWDOffset)/2) +
                           m_FFWDOffset;

            m_FFWDResults[1] = (-Math.cos((Math.PI * (time - m_ReferenceTime_ms))/ m_FFWDPeriod_ms) + 1) *
                    ((target - m_PositionOffset)/2) +
                    m_PositionOffset;
        }

        m_LastCoefficient = coefficient;
        m_LastTarget      = target;

        return m_FFWDResults;
    }

    public double[] CalcFFWDPosition(double coefficient, double target, double time)
    {
        if (m_LastTarget != target)
        {
            m_PositionOffset = m_LastTarget;
            m_ReferenceTime_ms = time;
        }

        if ((time - m_ReferenceTime_ms) > m_FFWDPeriod_ms)
        {
            m_FFWDResults[0] = 0;
            m_FFWDResults[1] = target;
        }
        else
        {
            m_FFWDResults[0] = (Math.cos((Math.PI * (time - m_ReferenceTime_ms))/ m_FFWDPeriod_ms) + 1) *
                    ((coefficient)/2);

            m_FFWDResults[1] = (-Math.cos((Math.PI * (time - m_ReferenceTime_ms))/ m_FFWDPeriod_ms) + 1) *
                    ((target - m_PositionOffset)/2) +
                    m_PositionOffset;
        }

        m_LastTarget = target;

        return m_FFWDResults;
    }

}
