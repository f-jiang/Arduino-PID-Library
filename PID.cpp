/**********************************************************************************************
 * Arduino PID Library - Version 1.1.1
 * by Brett Beauregard <br3ttb@gmail.com> brettbeauregard.com
 *
 * This Library is licensed under a GPLv3 License
 **********************************************************************************************/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include "PID.h"

/*Constructor (...)*********************************************************
 *    The parameters specified here are those for for which we can't set up
 *    reliable defaults, so we need to have the user set them.
 ***************************************************************************/
PID::PID(double& input, double& output, double& setpoint, double p, double i, double d) :
    m_input(input),
    m_output(output),
    m_setpoint(setpoint),
    m_isEnabled(false)
{
    PID::SetOutputLimits(0, 255);
    PID::SetTunings(p, i, d);

    m_sampleTime = 100;
    m_prevTime = millis() - m_sampleTime;
}

/* Compute() **********************************************************************
 *     This, as they say, is where the magic happens.  this function should be called
 *   every time "void loop()" executes.  the function will decide for itself whether a new
 *   pid Output needs to be computed.  returns true when the output is computed,
 *   false when nothing has been done.
 **********************************************************************************/
bool PID::Compute()
{
    if (!m_isEnabled) {
        return false;
    }

    unsigned long now = millis();

    if (now - m_prevTime >= m_sampleTime) {
        Clamp(m_input, m_inMin, m_inMax);

        double error = m_setpoint - m_input;

        m_iTerm += m_i * error;
        Clamp(m_iTerm, m_outMin, m_outMax);

        double output = m_p * error + m_iTerm - m_d * (m_input - m_prevInput);
        Clamp(output, m_outMin, m_outMax);
        m_output = output;

        m_prevInput = m_input;
        m_prevTime = now;

        return true;
    } else {
        return false;
    }
}

/* SetTunings(...)*************************************************************
 * This function allows the controller's dynamic performance to be adjusted.
 * it's called automatically from the constructor, but tunings can also
 * be adjusted on the fly during normal operation
 ******************************************************************************/
void PID::Enable()
{
    if (!IsEnabled()) {
        m_isEnabled = true;
        Initialize();
    }
}

void PID::Disable()
{
    m_isEnabled = false;
}

bool PID::IsEnabled() const
{
    return m_isEnabled;
}

void PID::SetTunings(double p, double i, double d)
{
    if (p < 0 || i < 0 || d < 0) {
        return;
    }

    m_dispP = p;
    m_dispI = i;
    m_dispD = d;

    double time = (double) m_sampleTime / 1000;
    m_p = p;
    m_i = i * time;
    m_d = d / time;
}

/* SetSampleTime(...) *********************************************************
 * sets the period, in Milliseconds, at which the calculation is performed	
 ******************************************************************************/
void PID::SetInputLimits(double min, double max)
{
    if (min >= max) {
        return;
    }

    m_inMin = min;
    m_inMax = max;

    if (m_isEnabled) {
        Clamp(m_input, m_inMin, m_inMax);
    }
}

void PID::SetOutputLimits(double min, double max)
{
    if (min >= max) {
        return;
    }

    m_outMin = min;
    m_outMax = max;

    if (m_isEnabled) {
        Clamp(m_output, m_outMin, m_outMax);
        Clamp(m_iTerm, m_outMin, m_outMax);
    }
}

void PID::SetSampleTime(unsigned long val)
{
    double ratio = (double) val / (double) m_sampleTime;
    m_i *= ratio;
    m_d /= ratio;
    m_sampleTime = val;
}

unsigned long PID::GetSampleTime() const
{
    return m_sampleTime;
}

/* SetOutputLimits(...)****************************************************
 *     This function will be used far more often than SetInputLimits.  while
 *  the input to the controller will generally be in the 0-1023 range (which is
 *  the default already,)  the output will be a little different.  maybe they'll
 *  be doing a time window and will need 0-8000 or something.  or maybe they'll
 *  want to clamp it from 0-125.  who knows.  at any rate, that can all be done
 *  here.
 **************************************************************************/
/* Initialize()****************************************************************
 *	does all the things that need to happen to ensure a bumpless transfer
 *  from manual to automatic mode.
 ******************************************************************************/
/* Status Funcions*************************************************************
 * Just because you set the Kp=-1 doesn't mean it actually happened.  these
 * functions query the internal state of the PID.  they're here for display
 * purposes.  this are the functions the PID Front-end uses for example
 ******************************************************************************/
double PID::GetP() const
{
    return m_dispP;
}

double PID::GetI() const
{
    return m_dispI;
}

double PID::GetD() const
{
    return m_dispD;
}

void PID::Initialize()
{
    m_iTerm = m_output;
    m_prevInput = m_input;
    Clamp(m_iTerm, m_outMin, m_outMax);
}

void PID::Clamp(double& val, double min, double max)
{
    if (val > max) {
        val = max;
    } else if (val < min) {
        val = min;
    }
}
