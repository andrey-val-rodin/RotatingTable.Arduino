#include <ESP32Encoder.h>

#define MOTOR 14
#define MOTOR_CHANNEL 0
#define DIRECTION 13
#define MOTOR_POWER 32
#define MOTOR_ENC1 33
#define MOTOR_ENC2 25
#define CAMERA 26
#define SHUTTER 27
#define CAMERA_LOW LOW
#define CAMERA_HIGH HIGH
#define MIN_PWM 97
#define MAX_PWM 188
#define GRADUATIONS 4320 // number of graduations per turn
#define DEGREE (GRADUATIONS / 360)

//#define DEBUG_MODE

ESP32Encoder encoder;

const unsigned char stepsLength = 22;
const uint16_t steps[stepsLength] =
    { 2, 4, 5, 6, 8, 9, 10, 12, 15, 18, 20, 24, 30, 36, 40, 45, 60, 72, 90, 120, 180, 360 };
signed char FindInSteps(uint16_t numberOfSteps)
{
    for (unsigned char i = 0; i < stepsLength; i++)
    {
        if (steps[i] == numberOfSteps)
            return i;
    }

    return -1;
}

void writeMotorPWM(unsigned char pwm, bool forward = true)
{
    ledcWrite(MOTOR_CHANNEL, pwm);
    digitalWrite(DIRECTION, forward? HIGH : LOW );
}

class Runner
{
    public:
        static inline bool isRunning()
        {
            return _isRunning;
        }
        
        static void runAutomatic();
        
    private:
        enum State : char
        {
            Waiting,
            Beginning,
            Delay,
            Exposure,
            Move
        };

        static int16_t _stepNumber;
        static unsigned long _timer;
        static unsigned long _timer2;
        static bool _isRunning;
        static int32_t _lastGraduations;
        static State _currentState;
        
        static void finalize();
        static void incrementStep();
        static void display(String top, String stepName);
};
Runner runner;

int16_t Runner::_stepNumber = 0;
unsigned long Runner::_timer = 0;
unsigned long Runner::_timer2 = 0;
bool Runner::_isRunning = false;
int32_t Runner::_lastGraduations;
Runner::State Runner::_currentState;

struct MenuItem
{
    String top;
    String bottom;
};

typedef void (*callback_t)();
struct MenuItemsDef
{
    static const unsigned char topItemsLength = 6;
    static MenuItem topItems[topItemsLength];

    static const unsigned char settingsItemsLength = 4;
    static MenuItem settingsItems[settingsItemsLength];

    static const callback_t handlers[1];
};

MenuItem MenuItemsDef::topItems[topItemsLength] = {
    {"%Auto",         ""},
    {"%Manual",       ""},
    {"%Nonstop",      ""},
    {"Video",         ""},
    {"Rotate 90\337", ""},
    {"Settings",      ""}
};
MenuItem MenuItemsDef::settingsItems[settingsItemsLength] = {
    {"Steps",        ""},
    {"Acceleration", ""},
    {"Delay",        ""},
    {"Exposure",     ""}
};
const callback_t MenuItemsDef::handlers[1] = {
    Runner::runAutomatic
};

class PWMValidator
{
    public:
        static unsigned char validate(int pwm)
        {
            if (pwm > MAX_PWM)
                pwm = MAX_PWM;
            else if (pwm < MIN_PWM)
                pwm = MIN_PWM;

            return pwm;
        }
};

class Settings
{
    public:
        static uint16_t getSteps()
        {
            return validateSteps(_steps);
        }

        static bool checkSteps(uint16_t value)
        {
            return FindInSteps(value) >= 0;
        }

        static uint16_t validateSteps(uint16_t value)
        {
            return checkSteps(value)
                ? value
                : 24; // use default
        }

        static void setSteps(uint16_t value)
        {
            _steps = value;
        }

        static unsigned char getAcceleration()
        {
            return validateAcceleration(_acceleration);
        }

        // Returns number of graduations for acceleration and deceleration from min to max PWM and vice versa.
        // Function converts current user-friendly value 1-10 to this value.
        static uint16_t getRealAcceleration()
        {
            switch(getAcceleration())
            {
                case 1:
                    return 550;
                case 2:
                    return 500;
                case 3:
                    return 450;
                case 4:
                    return 400;
                case 5:
                    return 350;
                case 6:
                    return 300;
                case 7:
                    return 250;
                case 8:
                    return 200;
                case 9:
                    return 150;
                default:
                    return 100;
            }
        }

        static bool checkAcceleration(unsigned char value)
        {
            return 1 <= value && value <= 10;
        }

        static unsigned char validateAcceleration(unsigned char value)
        {
            return checkAcceleration(value)
                ? value
                : 7; // use default
        }

        static void setAcceleration(unsigned char value)
        {
            _acceleration = value;
        }

        static uint16_t getDelay()
        {
            return 0;
        }

        static uint16_t getExposure()
        {
            return 100;
        }

    private:
        static uint16_t _steps;
        static unsigned char _acceleration;
};
uint16_t Settings::_steps;
unsigned char Settings::_acceleration;

class Mover
{
    public:
        enum State : char
        {
            Stop,
            Move,
            Stopping,
            Correction,
            RunAcc,
            Run,
            RunDec
        };

        void tick()
        {
            switch (_state)
            {
                case Move:
                case Stopping:
                case Correction:
                    tickMove();
                    break;
                    
                case Run:
                case RunAcc:
                case RunDec:
                    tickRun();
                    break;

                default:
                    break;
            }
        }

        State getState()
        {
            return _state;
        }

        void move(int32_t graduations, int maxPWM = MAX_PWM)
        {
            if (!isStopped())
                return;

            _graduations = graduations;
            _forward = graduations > 0;
            _currentPWM = MIN_PWM;
            _minPWM = MIN_PWM;
            _maxPWM = maxPWM;
            _cumulativePos -= encoder.getCount();
            encoder.setCount(0);
            _acceleration = Settings::getAcceleration();
            _realAcceleration = Settings::getRealAcceleration();
            _startTimer2 = _startTimer = millis();
            _started = false;
            _state = Move;

            writeMotorPWM(_currentPWM, _forward);
        }

        void run(int pwm)
        {
            if (!isStopped())
                return;

            _minPWM = MIN_PWM;
            _maxPWM = abs(pwm);
            _forward = pwm > 0;
            _currentPWM = MIN_PWM;
            _cumulativePos -= encoder.getCount();
            encoder.setCount(0);
            _acceleration = Settings::getAcceleration();
            _realAcceleration = Settings::getRealAcceleration();
            _startTimer2 = _startTimer = millis();
            _started = false;
            _state = RunAcc;

            writeMotorPWM(_currentPWM, _forward);
        }

        void stop()
        {
            writeMotorPWM(0);
            _state = Stop;
        }

        void softStop()
        {
            if (_state == Run)
            {
                // Calculate stop point
                float decelerationLength = _realAcceleration;
                float graduationsToStop = (_currentPWM - MIN_PWM) * decelerationLength /
                    (MAX_PWM - MIN_PWM);
                if (!_forward)
                    graduationsToStop = -graduationsToStop;
                _graduations = getCurrentPos() + graduationsToStop;
                _state = RunDec; // deceleration state
            }
            else
                stop();
        }

        inline bool isStopped()
        {
            return _state == Stop;
        }

        // Returns direction
        inline bool isForward()
        {
            return _forward;
        }

        // Returns current PWM
        inline int getCurrentPWM()
        {
            return _currentPWM;
        }

        // Returns maximum PWM
        inline int getMaxPWM()
        {
            return _maxPWM;
        }

        void changePWM(int delta)
        {
            if (!canChangePWM(delta))
                return;
                
            _maxPWM += delta;
            _maxPWM = PWMValidator::validate(_maxPWM);
            _currentPWM = _maxPWM;
            writeMotorPWM(_currentPWM, _forward);
        }

        bool canChangePWM(int delta)
        {
            // Changing is avalable only when state is Move or Run
            if (_state != Move && _state != Run)
                return false;

            // Changing is not awailable at the time of acceleration/deceleration
            if (!isUniformMotion())
                return false;

            int oldMaxPWM = _maxPWM;
            int newMaxPWM = PWMValidator::validate(_maxPWM + delta);

            // Return true if old _maxPWM will not be equal to the new one
            return newMaxPWM != oldMaxPWM;
        }

        inline bool isUniformMotion()
        {
            return (_state == Move || _state == Run) && _currentPWM == _maxPWM;
        }

        // Returns current position in graduations. Can be negative
        int32_t getCurrentPos()
        {
            int32_t pos = encoder.getCount();
            // Invert pos
            // clockwise movement is positive, counterclockwise movement is negative
            return -pos;
        }

        // Returns latest position obtained by Mover class.
        // This function is "easy" and does not use encoder and therefore critical section. 
        inline int32_t getLatestPos()
        {
            return _currentPos;
        }

        // Returns graduation count passed from starting point. Can be negative
        int32_t getAbsolutePos()
        {
            // Invert pos
            return _cumulativePos - encoder.getCount();
        }

        void resetAbsolutePos()
        {
            encoder.setCount(0);
            _cumulativePos = 0;
        }

    private:
        State _state = Stop;
        int32_t _graduations;
        bool _forward;
        int _minPWM;
        int _maxPWM;
        int32_t _currentPos;
        int32_t _lastPos;
        int32_t _cumulativePos;
        int _currentPWM;
        unsigned long _timer;
        unsigned long _startTimer;
        unsigned long _startTimer2;
        bool _started;
        unsigned char _timePartCount;
        int _acceleration;
        int _realAcceleration;
        
        bool start()
        {
            if (_started)
            {
                // Everything is OK, table started
                return true;
            }
           
            if (_currentPos != 0)
            {
                // Everything is OK, table started
                _started = true;
                return true;
            }

            // 100 ms is reasonable amount of time to start moving
            if (millis() - _startTimer2 >= 100)
            {
                int limit = calcHighLimitOfMinPWM();
                Serial.println("High limit of _minPWM: " + String(limit));
                if (_minPWM >= limit)
                {
                    // Unable to increase _minPWM anymore
                    // If correction, wait a second, and if table is still not moving, stop it
                    if (_state == Correction && millis() - _startTimer >= 1000)
                    {
                        Serial.println("Unable to start, stopping...");
                        stop();
                    }
                    return false;
                }

                _startTimer2 = millis();
                _minPWM += 1;
                if (_minPWM > limit)
                    _minPWM = limit;
                _currentPWM = _minPWM;
                writeMotorPWM(_currentPWM, _forward);
                Serial.println("Increase PWM. New value: " + String(_minPWM));
            }

            return false;
        }

        int calcHighLimitOfMinPWM()
        {
            int highestLimit = min(100, _maxPWM);
            
            switch (_state)
            {
                case RunAcc:
                    // In this case we can return a large enough value
                    return highestLimit;

                case Move:
                case Correction:
                    // Calculate maximum possible value
                    {
                        float halfPoint = abs(_graduations) / 2.0;
                        float accelerationLength = _realAcceleration;
                        float value = MIN_PWM + halfPoint * (MAX_PWM - MIN_PWM) / accelerationLength;
                        int result = validatePWM(value);
                        if (result > highestLimit)
                            result = highestLimit;

                        return result;
                    }

                default:
                    return MIN_PWM;
            }
        }
        
        void tickMove()
        {
            _currentPos = getCurrentPos();
            if (!start())
                return;

            bool reached = _forward
                ? _currentPos >= _graduations
                : _currentPos <= _graduations;
            if (reached)
            {
                switch (_state)
                {
                    case Move:
                        stop();
                        _state = Stopping;
                        _timePartCount = 0;
                        _lastPos = _currentPos;
                        _timer = millis();
                        return;

                    case Correction:
                        // finish
                        stop();
                        return;

                    default:
                        // continue
                        break;
                }
            }
            else if (_state == Move || _state == Correction)
            {
                bool firstHalf = _forward
                    ? _currentPos < _graduations / 2
                    : _currentPos > _graduations / 2;
                if (firstHalf)
                    accelerate();
                else
                    decelerate();

                writeMotorPWM(_currentPWM, _forward);
                return;
            }

            // Wait until table die
            if (_state == Stopping && millis() - _timer >= 10)
            {
                if (_currentPos == _lastPos)
                {
                    _timePartCount++;
                }
                else
                {
                    _timePartCount = 0;
                    _lastPos = _currentPos;
                }

                if (_timePartCount >= 10)
                {
                    // Table has been at rest for the last 10 times
                    makeCorrection();
                }
                else
                {
                    // continue
                    _timer = millis();
                }
            }
        }

        void accelerate()
        {
            // Use linear function to accelerate
            float x = _currentPos;
            if (!_forward)
                x = -x;
            float accelerationLength = _realAcceleration;
            float currentPWM = _minPWM + x * (MAX_PWM - _minPWM) / accelerationLength;
            _currentPWM = validatePWM(currentPWM);
        }

        void decelerate()
        {
            // Use linear function to decelerate
            float x = _graduations - _currentPos;
            if (!_forward)
                x = -x;
            x -= getFinalDistance();
            float decelerationLength = _realAcceleration;
            float currentPWM = _minPWM + x * (MAX_PWM - _minPWM) / decelerationLength;
            _currentPWM = validatePWM(currentPWM);
        }

        void makeCorrection()
        {
            int error = _graduations - _currentPos;
            if (error == 0)
            {
                // No correction needed, finish
                stop();
                return;
            }
            else
            {
                // Make correction
                Serial.println("Error = " + String(error) + ", correction...");
                stop();
                move(error, MIN_PWM);
                _state = Correction;
            }
        }

        // We should pass end of step with minimal PWM
        // This function returns length of this final distance
        int getFinalDistance()
        {
            switch (_acceleration)
            {
                case 10:
                    return _graduations >= 10 * DEGREE ? 18 : 14;
                case 9:
                    return _graduations >= 10 * DEGREE ? 10 : 8;
                case 8:
                    return _graduations >= 10 * DEGREE ? 6 : 4;
                case 7:
                    return 2;
                default:
                    return 0;
            }
        }
        
        void tickRun()
        {
            _currentPos = getCurrentPos();
            if (!start())
                return;

            switch (_state)
            {
                case RunAcc:
                    accelerate();
                    if (_currentPWM >= _maxPWM)
                    {
                        _currentPWM = _maxPWM;
                        _state = Run;
                    }
                    writeMotorPWM(_currentPWM, _forward);
                    break;
                    
                case RunDec:
                    decelerate();
                    if (_currentPWM <= _minPWM)
                        stop();
                    else
                        writeMotorPWM(_currentPWM, _forward);
                    break;

                default:
                    break;
            }
        }

        int validatePWM(int pwm)
        {
            if (pwm > _maxPWM)
                pwm = _maxPWM;
            else if (pwm < _minPWM)
                pwm = _minPWM;

            return pwm;
        }
};
Mover mover;

void Runner::runAutomatic()
{
    const char* mode = "Auto...";
    const char* stepName = "photo";
#ifdef DEBUG_MODE
    static int total = 0;
#endif
    
    if (!mover.isStopped())
        return;

    int16_t stepCount = Settings::getSteps();
    int16_t stepGraduations = GRADUATIONS / stepCount;
    if (!_isRunning)
    {
        // Starting
        _stepNumber = 0;
        digitalWrite(CAMERA, CAMERA_HIGH); // prepare camera
        _isRunning = true;
        _lastGraduations = 0;
        mover.resetAbsolutePos();
        _timer = millis();
        _currentState = Beginning;
#ifdef DEBUG_MODE
        total = 0;
#endif
        return;
    }

    if (_currentState == Beginning && millis() - _timer >= 10) // 10 ms is a time for preparing camera
    {
        _stepNumber++;
        display(mode, stepName);
        digitalWrite(SHUTTER, CAMERA_HIGH); // make first photo
        _timer = millis();
        _currentState = Exposure;
        return;
    }
    
    if (_currentState == Exposure && millis() - _timer >= Settings::getExposure())
    {
        digitalWrite(SHUTTER, CAMERA_LOW); // release shutter
        _currentState = Move;
        int32_t absolutePos = mover.getAbsolutePos();
        int error = _lastGraduations - absolutePos;
#ifdef DEBUG_MODE
        total += absolutePos;
        Serial.println("stepGraduations = " + String(stepGraduations) + "  absolutePos = " +
            String(absolutePos) + "  error = " + String(error));
#endif
        mover.resetAbsolutePos();
        _lastGraduations = stepGraduations + error;
        mover.move(_lastGraduations);
        return;
    }
    
    if (_currentState == Move)
    {
        if (_stepNumber < stepCount)
        {
            _timer = millis();
            _currentState = Delay;
        }
        else
        {
#ifdef DEBUG_MODE
            int32_t absolutePos = mover.getAbsolutePos();
            total += absolutePos;
            Serial.println("Total = " + String(total));
#endif
            finalize();
        }
        return;
    }

    if (_currentState == Delay && millis() - _timer >= Settings::getDelay())
    {
        _stepNumber++;
        display(mode, stepName);
        digitalWrite(SHUTTER, CAMERA_HIGH); // make photo
        _timer = millis();
        _currentState = Exposure;
    }
}

void Runner::finalize()
{
    _isRunning = false;
    mover.stop();
    _stepNumber = 0;
    digitalWrite(SHUTTER, CAMERA_LOW); // release shutter
    digitalWrite(CAMERA, CAMERA_LOW); // release camera
}

void Runner::display(String, String)
{
}

enum State
{
    Waiting,
    Measuring,
    Measured
};

class TurnMeasurer
{
    public:
        inline State getState()
        {
            return _state;
        }

        void tick()
        {
            switch (_state)
            {
                case Waiting:
                case Measured:
                    return;
    
                case Measuring:
                    Runner::runAutomatic();
                    if (Runner::isRunning())
                        return;

                    _stop = millis();
                    _state = Measured;
            }
        }
    
        void measure(int acceleration)
        {
            Settings::setAcceleration(acceleration);
            _state = Measuring;
            _start = millis();
        }
    
        String getOutput()
        {
            float time = (_stop - _start) / 1000.0;
            return "\t" + String(time);
        }
    
        void cancel()
        {
            if (_state == Measuring)
            {
                mover.stop();
            }
    
            _state = Waiting;
        }

    private:
        State _state = Waiting;
        unsigned long _start;
        unsigned long _stop;
};

class Measurer
{
    public:
        inline State getState()
        {
            return _state;
        }

        void tick()
        {
            _measurer.tick();
            
            switch (_state)
            {
                case Measuring:
                    if (_measurer.getState() == Measured)
                    {
                        Serial.print(_measurer.getOutput());
                        _acceleration++;
                        if (_acceleration > 10)
                        {
                            _state = Measured;
                            return;
                        }
                        else
                        {
                            _measurer.measure(_acceleration);
                        }
                    }
                    return;

                default:
                    return;
            }
        }

        void measure(int fromAcc)
        {
            _acceleration = fromAcc;
            _state = Measuring;
            Serial.print(Settings::getSteps());
            _measurer.measure(_acceleration);
        }
    
        void cancel()
        {
            _measurer.cancel();
            _state = Waiting;
        }
    
    private:
        State _state = Waiting;
        TurnMeasurer _measurer;
        int _acceleration;
};

class Worker
{
    public:
        inline State getState()
        {
            return _state;
        }
        
        void tick()
        {
            if (_state == Waiting)
                return;
                
            _measurer.tick();

            if(_measurer.getState() == Measured)
            {
                _stepIndex++;
                if (_stepIndex > _stepIndexTo)
                {
                    _state = Waiting;
                    return;
                }

                Serial.println();
                Settings::setSteps(steps[_stepIndex]);
                _measurer.measure(_fromAcc);
            }
        }

        void start(int fromAcc = 1, int stepIndexFrom = 0, int stepIndexTo = stepsLength - 1)
        {
            _fromAcc = fromAcc;
            _stepIndexFrom = stepIndexFrom;
            _stepIndexTo = stepIndexTo;
            _state = Measuring;
            _stepIndex = _stepIndexFrom;
            Settings::setSteps(steps[_stepIndex]);
            Settings::setAcceleration(1);
            for (int i = fromAcc; i <= 10; i++)
            {
                Serial.print("\t");
                Serial.print(i);              
            }
            Serial.println();
            _measurer.measure(_fromAcc);
        }

        void cancel()
        {
            _measurer.cancel();
            _state = Waiting;
        }
        
    private:
        State _state = Waiting;
        Measurer _measurer;
        unsigned char _stepIndex;
        unsigned char _stepIndexFrom;
        unsigned char _stepIndexTo;
        int _fromAcc;
};
Worker worker;

void setup()
{
    ESP32Encoder::useInternalWeakPullResistors = NONE;
    encoder.setFilter(0);
    encoder.always_interrupt = true;

    // use pin 33 and 25 for the encoder
    encoder.attachFullQuad(33, 25);
    encoder.clearCount();

    ledcAttachPin(MOTOR, MOTOR_CHANNEL);
    // frequency 20000 Hz, resolution 8 bit
    ledcSetup(MOTOR_CHANNEL, 20000, 8);
    
    pinMode(DIRECTION, OUTPUT);
    pinMode(MOTOR_POWER, OUTPUT);
    pinMode(CAMERA, OUTPUT);
    pinMode(SHUTTER, OUTPUT);
    digitalWrite(SHUTTER, CAMERA_LOW); // release shutter
    digitalWrite(CAMERA, CAMERA_LOW); // release camera
    writeMotorPWM(0);
    digitalWrite(MOTOR_POWER, HIGH);

    Serial.begin(115200);

    delay(500);
    worker.start();
}

void loop()
{
    mover.tick();
    worker.tick();
}
