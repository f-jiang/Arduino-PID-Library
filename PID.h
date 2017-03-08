#ifndef PID_H
#define PID_H

class PID {
public :
    PID(double&, double&, double&, double, double, double);

    bool Compute();

    void Enable();
    void Disable();
    bool IsEnabled() const;

    void SetTunings(double, double, double);

    void SetInputLimits(double, double);
    void SetOutputLimits(double, double);

    void SetSampleTime(unsigned long);
    unsigned long GetSampleTime() const;

    double GetP() const;
    double GetI() const;
    double GetD() const;

private :
    bool m_isEnabled;
    double m_dispP;
    double m_dispI;
    double m_dispD;

    double m_p;
    double m_i;
    double m_d;

    double m_inMin;
    double m_inMax;
    double m_outMin;
    double m_outMax;

    double& m_input;
    double& m_output;
    double& m_setpoint;

    unsigned long m_sampleTime;
    unsigned long m_prevTime;
    double m_prevInput;

    double m_iTerm;

    void Initialize();
    void Clamp(double&, double, double);
};

#endif

