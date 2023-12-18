#include <mbed.h>
#include <math.h>
#define _USE_MATH_DEFINES

class Servo {
private:
    // PWM output
    PinName pin; 
    PwmOut* pwm;
    const float period = 0.02f;
    int min_pulse_width = 530;
    int max_pulse_width = 2400;
    
    // Angle range (in radians)
    double min_angle = 0.0;
    double max_angle = M_PI;
    const double home_angle = M_PI / 2.0;
    double angle = home_angle;

public:
    // Constructors
    Servo(PinName _pin);
    Servo(PinName _pin, float _period, int min_pulse, int max_pulse, double _home_angle);
    Servo(PinName _pin, float _period, int min_pulse, int max_pulse, double _home_angle, double delta);
    Servo(PinName _pin, float _period, int min_pulse, int max_pulse, double _home_angle, double _min_angle, double _max_angle);
    // Destructor
    ~Servo();

    // Move servo to appropriate angle. Maps from joint range [home_angle - delta, home_angle + delta] -> servo pwm range [min_pwm, max_pwm]
    void set_angle(double radians);

    void print(void);

    // user-input
    //void calibrate(void);
};