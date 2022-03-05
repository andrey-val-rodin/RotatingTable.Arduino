#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <PWM.h>

#define MOTOR1 10
#define MOTOR2 9
#define MOTOR_ENC1 2
#define MOTOR_ENC2 3
#define CAMERA 5
#define SHUTTER 6
#define CAMERA_LOW HIGH
#define CAMERA_HIGH LOW
#define MIN_PWM 60
#define MAX_PWM 255
#define GRADUATIONS 4320 // number of graduations per turn
#define DEGREE (GRADUATIONS / 360)

// Uncomment below to enable debug output.
//#define DEBUG_MODE

Encoder encoder(MOTOR_ENC1, MOTOR_ENC2);

bool UseBluetooth = true;

uint16_t EEMEM stepsOffset;
unsigned char EEMEM accelerationOffset;
uint16_t EEMEM delayOffset;
uint16_t EEMEM exposureOffset;
uint16_t EEMEM videoPWMOffset;
float EEMEM nonstopFrequencyOffset;
unsigned char EEMEM menuIndexOffset;

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

char strBuf[30];
const char terminator = '\n';
void Write(const String& text)
{
    sprintf(strBuf, "%s%c", text.c_str(), terminator);
    Serial.write(strBuf);
}

class Runner
{
    public:
        static const int delta = 5; // value to increment/decrement pwm

        enum Mode
        {
            None,
            Auto,
            Manual,
            Nonstop,
            Rotate,
            Video,
            FreeMovement
        };

        static void runAutomatic();
        static void runManual();
        static void runNonstop();
        static void runVideo();
        static void runRotate();
        static void runFreeMovement();

        static void stop();
        static inline bool isStopping();
        static inline bool isIncreasePWM();
        static inline bool isDecreasePWM();
        static inline bool isChangingPWM();
        static int setChangingPWM(int value);
        static inline bool isRunning();
        static inline bool isBusy();
        static inline Mode getMode();
        static void run(Mode mode);
        static void rotate(int pos);
        static void tick();

    private:
        enum State : char
        {
            Waiting,
            Beginning,
            Delay,
            Exposure,
            Move
        };

        static Mode _mode;
        static int16_t _stepNumber;
        static unsigned long _timer;
        static bool _isRunning;
        static bool _isBusy;
        static bool _stop;
        static State _currentState;
        static int32_t _lastGraduations;
        static bool _needToMove;
        static int _nextSnapshotPos;
        static bool _needToStoreNewPWM;
        static int _currentAngle;
        static int _oldAngle;
        static int _changePWM;
    
        static void finalize();
        static char* format(char* format, int arg);
        static void display(const String& top, const String& stepName);
};

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

    static const callback_t handlers[topItemsLength - 1];
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
    {"Steps",         ""},
    {"Acceleration",  ""},
    {"Delay",         ""},
    {"Exposure",      ""}
};
const callback_t MenuItemsDef::handlers[topItemsLength - 1] = {
    Runner::runAutomatic,
    Runner::runManual,
    Runner::runNonstop,
    Runner::runVideo,
    Runner::runRotate
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

#ifdef DEBUG_MODE
uint16_t lastLowSteps = 0;
uint16_t lastHighSteps = 0;
#endif
class Settings
{
    public:
        static constexpr float lowNonstopFrequency = 0.5;
        static constexpr float highNonstopFrequency = 3.0;
        
        static uint16_t getSteps()
        {
            return validateSteps(eeprom_read_word(&stepsOffset));
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
            eeprom_update_word(&stepsOffset, value);
        }

        static unsigned char getAcceleration()
        {
            return validateAcceleration(eeprom_read_byte(&accelerationOffset));
        }

        // Returns number of graduations for acceleration and deceleration from min to max PWM and vice versa.
        // Function converts current user-friendly value 1-10 to this value.
        // Note that real value shouldn't be less than 80 when GRADUATIONS = 4320.
        static uint16_t getRealAcceleration()
        {
            int result = getAcceleration();
            result = abs(result - 11); // reverse
            result *= 10;
            result += 10;
            result *= DEGREE;
            result /= 3;
            return result; // value in range from 80 to 440 when GRADUATIONS = 4320
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
            eeprom_update_byte(&accelerationOffset, value);
        }

        static uint16_t getDelay()
        {
            return validateDelay(eeprom_read_word(&delayOffset));
        }

        static bool checkDelay(uint16_t value)
        {
            return value <= 5000 && value % 100 == 0;
        }

        static uint16_t validateDelay(uint16_t value)
        {
            return checkDelay(value)
                ? value
                : 0; // use default
        }

        static void setDelay(uint16_t value)
        {
            eeprom_update_word(&delayOffset, value);
        }

        static uint16_t getExposure()
        {
            return validateExposure(eeprom_read_word(&exposureOffset));
        }

        static bool checkExposure(uint16_t value)
        {
            return 100 <= value && value <= 500 && value % 100 == 0;
        }

        static uint16_t validateExposure(uint16_t value)
        {
            return checkExposure(value)
                ? value
                : 100; // use default
        }

        static void setExposure(uint16_t value)
        {
            eeprom_update_word(&exposureOffset, value);
        }

        static int16_t getVideoPWM()
        {
            return validateVideoPWM(eeprom_read_word(&videoPWMOffset));
        }

        static int16_t validateVideoPWM(int16_t value)
        {
            return 
                (-MAX_PWM <= value && value <= -MIN_PWM) ||
                ( MIN_PWM <= value && value <=  MAX_PWM)
                    ? value
                    : 100; // use default
        }

        static void setVideoPWM(int16_t value)
        {
            eeprom_update_word(&videoPWMOffset, value);
        }

        static float getNonstopFrequency()
        {
            return validateNonstopFrequency(eeprom_read_float(&nonstopFrequencyOffset));
        }

        static float validateNonstopFrequency(float value)
        {
            return lowNonstopFrequency <= value && value <= highNonstopFrequency
                ? value
                : lowNonstopFrequency; // use default
        }
        
        static unsigned char getRealNonstopPWM()
        {
            float frequency = getNonstopFrequency();
            unsigned char result = frequencyToPWM(frequency);
#ifdef DEBUG_MODE
            Serial.println("frequency from EEPROM = " + String(frequency) + "\tpwm = " + String(result));
#endif
            return PWMValidator::validate(result);
        }

        static unsigned char getLowRealNonstopPWM()
        {
            float frequency = lowNonstopFrequency;
            unsigned char result = PWMValidator::validate(frequencyToPWM(frequency));
#ifdef DEBUG_MODE
            if (lastLowSteps != getSteps())
            {
                Serial.println("low pwm = " + String(result));
                lastLowSteps = getSteps();
            }
#endif
            return result;
        }

        static unsigned char getHighRealNonstopPWM()
        {
            float frequency = highNonstopFrequency;
            unsigned char result = PWMValidator::validate(frequencyToPWM(frequency));
#ifdef DEBUG_MODE
            if (lastHighSteps != getSteps())
            {
                Serial.println("high pwm = " + String(result));
                lastHighSteps = getSteps();
            }
#endif
            return result;
        }

        static void setNonstopFrequency(float value)
        {
#ifdef DEBUG_MODE
            Serial.println("Store frequency in EEPROM = " + String(value));
#endif
            eeprom_update_float(&nonstopFrequencyOffset, value);
        }
        
        static void setNonstopPWM(unsigned char value)
        {
            float frequency = pwmToFrequency(value);
#ifdef DEBUG_MODE
            Serial.println("Store frequency = " + String(frequency));
#endif
            setNonstopFrequency(frequency);
        }

        static unsigned char getMenuIndex()
        {
            return validateMenuIndex(eeprom_read_byte(&menuIndexOffset));
        }

        static unsigned char validateMenuIndex(unsigned char value)
        {
            return value <= MenuItemsDef::topItemsLength
                ? value
                : 0; // use default
        }

        static void setMenuIndex(unsigned char value)
        {
            eeprom_update_byte(&menuIndexOffset, value);
        }

    private:
        static float getTimeOfTurn(unsigned char pwm)
        {
            static const unsigned char buff[] =
            {
                73, 69, 63, 60, 56, 52, 49, 46, 44, 41, 39, 38, 36, 35, 34, 32, 31, 30, 29, 28, 27, 26,
                25, 25, 24, 24, 24, 24, 23, 23, 22, 22, 22, 21, 21, 20, 20, 20, 19, 19, 19, 19, 18, 18,
                18, 18, 17, 17, 17, 17, 17, 16, 16, 16, 16, 16, 16, 15, 15, 15, 15, 15, 15
            };
            
            int index = pwm - MIN_PWM;
            if (index < 63)
                return buff[index];
            else if (index < 72)
                return 14;
            else if (index < 81)
                return 13;
            else if (index < 90)
                return 12;
            else if (index < 101)
                return 11;
            else if (index < 112)
                return 10;
            else if (index < 124)
                return 9;
            else if (index < 138)
                return 8;
            else
                return 7;
        }

        static float getPWMOfTurn(float time)
        {
            int pwm = MAX_PWM;
            while (getTimeOfTurn(pwm) < time && pwm > MIN_PWM)
            {
                pwm--;
            }

            return pwm;
        }

        static float pwmToFrequency(unsigned char pwm)
        {
            float steps = getSteps();
            return steps / getTimeOfTurn(pwm);
        }

        static unsigned char frequencyToPWM(float frequency)
        {
            float steps = getSteps();
            float time = steps / frequency;
            float result = getPWMOfTurn(time);
            return result + 0.5; // rounded
        }
};

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
            _maxPWM = maxPWM;
            _cumulativePos -= encoder.readAndReset();
            _acceleration = Settings::getAcceleration();
            _realAcceleration = Settings::getRealAcceleration();
            _state = Move;
        }

        void run(int pwm)
        {
            if (!isStopped())
                return;

            _maxPWM = abs(pwm);
            _forward = pwm > 0;
            _currentPWM = MIN_PWM;
            _cumulativePos -= encoder.readAndReset();
            _acceleration = Settings::getAcceleration();
            _realAcceleration = Settings::getRealAcceleration();
            _state = RunAcc;
        }

        void stop()
        {
            analogWrite(MOTOR1, 0);
            analogWrite(MOTOR2, 0);
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
            switch (_state)
            {
                case Move:
                    if (_currentPWM == _maxPWM)
                    {
                        _maxPWM += delta;
                        _maxPWM = PWMValidator::validate(_maxPWM);
                        _currentPWM = _maxPWM;
                        analogWrite(_forward? MOTOR1 : MOTOR2, _currentPWM);
                    }
                    break;
                    
                case Run:
                    _maxPWM += delta;
                    _maxPWM = PWMValidator::validate(_maxPWM);
                    _currentPWM = _maxPWM;
                    analogWrite(_forward? MOTOR1 : MOTOR2, _currentPWM);
                    break;

                default:
                    break;
            }
        }

        // Returns current position in graduations. Can be negative
        int32_t getCurrentPos()
        {
            int32_t pos = encoder.read();
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
            return _cumulativePos - encoder.read();
        }

        void resetAbsolutePos()
        {
            encoder.readAndReset();
            _cumulativePos = 0;
        }

    private:
        State _state = Stop;
        int32_t _graduations;
        bool _forward;
        int _maxPWM = MAX_PWM;
        int32_t _currentPos;
        int32_t _lastPos;
        int32_t _cumulativePos;
        int _currentPWM;
        unsigned long _timer;
        unsigned char _timePartCount;
        int _acceleration;
        int _realAcceleration;
        
        void tickMove()
        {
            _currentPos = getCurrentPos();
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

                analogWrite(_forward? MOTOR1 : MOTOR2, _currentPWM);
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
            float currentPWM = MIN_PWM + x * (MAX_PWM - MIN_PWM) / accelerationLength;
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
            float currentPWM = MIN_PWM + x * (MAX_PWM - MIN_PWM) / decelerationLength;
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
#ifdef DEBUG_MODE
                Serial.println("Error = " + String(error) + ", correction...");
#endif
                stop();
                move(error, MIN_PWM);
                _state = Correction;
            }
        }

        // We should pass end of step with MIN_PWM
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
            switch (_state)
            {
                case RunAcc:
                    _currentPos = getCurrentPos();
                    accelerate();
                    if (_currentPWM >= _maxPWM)
                    {
                        _currentPWM = _maxPWM;
                        _state = Run;
                    }
                    analogWrite(_forward? MOTOR1 : MOTOR2, _currentPWM);
                    break;
                    
                case RunDec:
                    _currentPos = getCurrentPos();
                    decelerate();
                    if (_currentPWM <= MIN_PWM)
                        stop();
                    else
                        analogWrite(_forward? MOTOR1 : MOTOR2, _currentPWM);
                    break;

                default:
                    break;
            }
        }

        int validatePWM(int pwm)
        {
            if (pwm > _maxPWM)
                pwm = _maxPWM;
            else if (pwm < MIN_PWM)
                pwm = MIN_PWM;

            return pwm;
        }
};
Mover mover;


// Runner
Runner::Mode Runner::_mode = None;
int16_t Runner::_stepNumber = 0;
unsigned long Runner::_timer = 0;
bool Runner::_isRunning = false;
bool Runner::_isBusy = false;
bool Runner::_stop = false;
Runner::State Runner::_currentState;
int32_t Runner::_lastGraduations;
bool Runner::_needToMove;
int Runner::_nextSnapshotPos;
bool Runner::_needToStoreNewPWM;
int Runner::_currentAngle;
int Runner::_oldAngle;
int Runner::_changePWM = 0;

void Runner::stop()
{
    _stop = true;
}

inline bool Runner::isStopping()
{
    return _stop;//    return UseBluetooth? _stop : enc.press();
}

inline bool Runner::isIncreasePWM()
{
    return _changePWM > 0;//    return UseBluetooth? _changePWM > 0 : enc.right();
}

inline bool Runner::isDecreasePWM()
{
    return _changePWM < 0;//    return UseBluetooth? _changePWM < 0 : enc.left();
}

inline bool Runner::isChangingPWM()
{
    return _changePWM != 0;//    return UseBluetooth? _changePWM != 0 : enc.turn();
}

int Runner::setChangingPWM(int value)
{
    _changePWM = value;
}

inline bool Runner::isRunning()
{
    return _isRunning;
}

inline bool Runner::isBusy()
{
    return _isBusy;
}

inline Runner::Mode Runner::getMode()
{
    return _mode;
}

void Runner::run(Mode mode)
{
    _mode = mode;
}

void Runner::rotate(int pos)
{
    _currentState = Beginning;
    mover.move(pos);
}

void Runner::tick()
{
    switch (_mode)
    {
        case Auto:
            runAutomatic();
            break;

        case Manual:
            runManual();
            break;

        case Nonstop:
            runNonstop();
            break;

//        case Rotate:
//            runRotate();
//            break;

        case Video:
            runVideo();
            break;

        case FreeMovement:
            runFreeMovement();
            break;
            
        default:
            break;
    }
}

void Runner::runAutomatic()
{
    const String mode = "Auto...";
    const String stepName = "photo";
#ifdef DEBUG_MODE
    static int total = 0;
#endif
    
    if (isStopping())
    {
        finalize();
        return;
    }

    if (!mover.isStopped())
    {
        if (UseBluetooth)
        {
            _currentAngle = mover.getLatestPos() / DEGREE;
            if (_currentAngle != _oldAngle)
            {
                _oldAngle = _currentAngle;
                Write(format("POS %d", _currentAngle));
            }
        }
        return;
    }

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
        if (UseBluetooth)
            Write(format("STEP %d", _stepNumber));
        else
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
        if (UseBluetooth)
            Write(format("STEP %d", _stepNumber));
        else
            display(mode, stepName);
	    
        digitalWrite(SHUTTER, CAMERA_HIGH); // make photo
        _timer = millis();
        _currentState = Exposure;
    }
}

void Runner::runManual()
{
    const String mode = "Manual...";
    const String stepName = "step";
#ifdef DEBUG_MODE
    static int total = 0;
#endif
    
    if (isStopping())
    {
        finalize();
        return;
    }

    int16_t stepCount = Settings::getSteps();
    int stepGraduations = GRADUATIONS / stepCount;
    if (!_isBusy)
    {
        // Starting
        _stepNumber = 1;
        _needToMove = false;
        if (!UseBluetooth)
            display(mode, stepName);
        digitalWrite(CAMERA, CAMERA_HIGH); // prepare camera
        _isBusy = true;
        _lastGraduations = 0;
        mover.resetAbsolutePos();
        _timer = millis();
        _currentState = Exposure;
#ifdef DEBUG_MODE
        total = 0;
#endif
        return;
    }

    if (_currentState == Exposure && millis() - _timer >= Settings::getExposure())
    {
        digitalWrite(SHUTTER, CAMERA_LOW); // release shutter
        _currentState = Waiting;
        if (_needToMove)
        {
            _needToMove = false;
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
        }

        return;
    }
/*
    if (photoButton.press() && _currentState == Waiting)
    {
        digitalWrite(SHUTTER, CAMERA_HIGH); // make photo
        _timer = millis();
        _currentState = Exposure;
        return;
    }

    if (nextButton.press() && _currentState == Waiting)
    {
        digitalWrite(SHUTTER, CAMERA_HIGH); // make photo
        _timer = millis();
        _currentState = Exposure;
        _needToMove = true;
        return;
    }
*/
    if (_currentState == Move && mover.isStopped())
    {
        if (_stepNumber < stepCount)
        {
            _currentState = Waiting;
            _stepNumber++;
            if (UseBluetooth)
                Write(format("STEP %d", _stepNumber));
            else
                display(mode, stepName);
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
    }
}

void Runner::runNonstop()
{
    const String mode = "Nonstop...";
    const String stepName = "photo";
#ifdef DEBUG_MODE
    static int total = 0;
#endif
/*
    if (isStopping())
    {
        finalize();
        return;
    }
    else if (enc.turn() && !mover.isStopped())
    {
        if (enc.left())
        {
            if (mover.getCurrentPWM() > Settings::getLowRealNonstopPWM())
            {
                int d = delta;
                if (mover.getCurrentPWM() - d < Settings::getLowRealNonstopPWM())
                    d = mover.getCurrentPWM() - Settings::getLowRealNonstopPWM();

                mover.changePWM(-d);
                _needToStoreNewPWM = true;
            }
        }
        else if (enc.right())
        {
            if (mover.getCurrentPWM() < Settings::getHighRealNonstopPWM())
            {
                int d = delta;
                if (mover.getCurrentPWM() + d > Settings::getHighRealNonstopPWM())
                    d = Settings::getHighRealNonstopPWM() - mover.getCurrentPWM();

                mover.changePWM(d);
                _needToStoreNewPWM = true;
            }
        }
    }

    int16_t stepCount = Settings::getSteps();
    int stepGraduations = GRADUATIONS / stepCount;
    if (!_isRunning)
    {
        // Starting
        _stepNumber = 0;
        digitalWrite(CAMERA, CAMERA_HIGH); // prepare camera
        _isRunning = true;
        _needToStoreNewPWM = false;
        _timer = millis();
        _currentState = Beginning;
#ifdef DEBUG_MODE
        total = 0;
        mover.resetAbsolutePos();
#endif
        return;
    }

    if (_currentState == Beginning && millis() - _timer >= Settings::getExposure())
    {
        _stepNumber++;
        if (UseBluetooth)
            Write(format("STEP %d", _stepNumber));
        else
            display(mode, stepName);
        digitalWrite(SHUTTER, CAMERA_HIGH); // make first photo
        _nextSnapshotPos = stepGraduations;
        _timer = millis();
        _currentState = Exposure;
        mover.move(GRADUATIONS, Settings::getRealNonstopPWM());
        return;
    }

    const int releaseShutterDelay = 50;
    if (_currentState == Exposure && millis() - _timer >= releaseShutterDelay)
    {
        digitalWrite(SHUTTER, CAMERA_LOW); // release shutter
        _currentState = Move;
    }

    if (_currentState == Move && mover.getCurrentPos() >= _nextSnapshotPos)
    {
        _nextSnapshotPos += stepGraduations;
        _stepNumber++;
        if (_stepNumber > stepCount)
        {
            _currentState = Waiting;
        }
        else
        {
            if (UseBluetooth)
                Write(format("STEP %d", _stepNumber));
            else
                display(mode, stepName);
            digitalWrite(SHUTTER, CAMERA_HIGH); // make photo
            _timer = millis();
            _currentState = Exposure;
        }
    }

    if (_isRunning && _currentState != Beginning && mover.isStopped())
    {
        if (_needToStoreNewPWM)
        {
#ifdef DEBUG_MODE
            Serial.println("Store new pwm = " + String(mover.getMaxPWM()));
#endif
            Settings::setNonstopPWM(mover.getMaxPWM());
            _needToStoreNewPWM = false;
        }

#ifdef DEBUG_MODE
        int32_t absolutePos = mover.getAbsolutePos();
        total += absolutePos;
        Serial.println("Total = " + String(total));
#endif
        finalize();
    }
    */
}

void Runner::runVideo()
{
    if (!_isRunning)
    {
//        if (!UseBluetooth)
//            selector.menu.display("Video...", "");
        mover.resetAbsolutePos();
        mover.run(Settings::getVideoPWM());
        _timer = millis();
        _isRunning = true;
        return;
    }

    if (mover.isStopped())
    {
        finalize();
        return;
    }
    
    char direction = mover.isForward() ? 1 : -1;
    if (isStopping())
    {
        if (mover.getState() == mover.State::Run)
        {
            Settings::setVideoPWM(mover.getMaxPWM() * direction);
            mover.softStop();
        }
        else
        {
            mover.stop();
        }
    }
    else if (mover.getState() == mover.State::Run)
    {
        if (UseBluetooth && millis() - _timer >= 50)
        {
            _currentAngle = mover.getAbsolutePos() / DEGREE;
            if (_currentAngle != _oldAngle)
            {
                _oldAngle = _currentAngle;
                Write(format("POS %d", _currentAngle));
            }

            _timer = millis();
        }
        
        if (isChangingPWM())
        {
            if (isDecreasePWM())
            {
                if (direction > 0 && mover.getCurrentPWM() <= MIN_PWM)
                {
                    // Change direction
                    mover.stop();
                    mover.run(-MIN_PWM);
                    return;
                }
    
                mover.changePWM(-delta * direction);
            }
            else if (isIncreasePWM())
            {
                if (direction < 0 && mover.getCurrentPWM() <= MIN_PWM)
                {
                    // Change direction
                    mover.stop();
                    mover.run(MIN_PWM);
                    return;
                }
    
                mover.changePWM(delta * direction);
            }

            setChangingPWM(0);
        }
    }
}

void Runner::runFreeMovement()
{
    if (!_isBusy)
    {
        _currentState = Waiting;
        _isBusy = true;
    }
    
    if (isStopping())
    {
        finalize();
        return;
    }

    if (_currentState == Beginning && mover.isStopped())
    {
        Write("END");
        _currentState = Waiting;
    }
    else
    {
        _currentAngle = mover.getLatestPos() / DEGREE;
        if (_currentAngle != _oldAngle)
//                if (abs(_currentAngle - _oldAngle) >= 10) // TODO temporary
        {
            _oldAngle = _currentAngle;
            Write(format("POS %d", _currentAngle));
        }
    }
}

void Runner::finalize()
{
    _isRunning = _isBusy = false;
    _stop = false;
    _mode = None;
    mover.stop();
    _stepNumber = 0;
    _currentAngle = _oldAngle = 0;
    _changePWM = 0;
    digitalWrite(SHUTTER, CAMERA_LOW); // release shutter
    digitalWrite(CAMERA, CAMERA_LOW); // release camera
    if (UseBluetooth)
        Write("END");
}

char* Runner::format(char* format, int arg)
{
    sprintf(strBuf, format, arg);
    return strBuf;
}

void Runner::display(const String& top, const String& stepName)
{
    char strBuf[20];
    sprintf(strBuf, "%s %d (%d)", stepName.c_str(), _stepNumber, Settings::getSteps());
//    selector.menu.display(top, strBuf);
}

class Listener
{
    public:
        const String Status           = "STATUS";
        const String GetSteps         = "GET STEPS";
        const String GetAcceleration  = "GET ACC";
        const String GetExposure      = "GET EXP";
        const String GetDelay         = "GET DELAY";
        const String SetAcceleration  = "SET ACC";
        const String SetSteps         = "SET STEPS";
        const String SetExposure      = "SET EXP";
        const String SetDelay         = "SET DELAY";
        const String GetPosition      = "GET POS";
        const String GetMode          = "GET MODE";
        const String IsRunning        = "IS RUNNING"; //TODO remove?
        const String RunAutoMode      = "RUN AUTO";
        const String RunManualMode    = "RUN MANUAL";
        const String RunNonStopMode   = "RUN NS";
        const String RunVideoMode     = "RUN VIDEO";
        const String RunRotateMode    = "RUN ROTATE";
        const String RunFreeMovement  = "RUN FM";
        const String Shutter          = "SHUTTER";
        const String Next             = "NEXT";
        const String Stop             = "STOP";
        const String IncreasePWM      = "INCPWM";
        const String DecreasePWM      = "DECPWM";
        const String Undefined        = "UNDEF";

        void tick()
        {
           if (Serial.available())
            {
                String command = Serial.readStringUntil(terminator);
                if (command == Status)
                {
                    if (Runner::isRunning())
                        Write("RUNNING");
                    else if (Runner::isBusy())
                        Write("BUSY");
                    else
                        Write("READY");
                }
                else if (command.startsWith("GET "))
                {
                    command.replace("GET ", "");
                    if (command == "STEPS")
                    {
                        Write(String(Settings::getSteps()));
                    }
                    else if (command == "ACC")
                    {
                        Write(String(Settings::getAcceleration()));
                    }
                    else if (command == "EXP")
                    {
                        Write(String(Settings::getExposure()));
                    }
                    else if (command == "DELAY")
                    {
                        Write(String(Settings::getDelay()));
                    }
                    else if (command == "POS")
                    {
                        //TODO
                    }
                    else if (command == "MODE")
                    {
                        Write(String(Runner::getMode()));
                    }
                }
                else if (command.startsWith("SET "))
                {
                    command.replace("SET ", "");
                    if (command.startsWith("STEPS "))
                    {
                        if (Runner::getMode() != Runner::None)
                        {
                            Write("ERR");
                        }
                        else
                        {
                            command.replace("STEPS ", "");
                            uint16_t value = command.toInt();
                            if (Settings::checkSteps(value))
                            {
                                Settings::setSteps(value);
                                // Check whether the value was successfully stored in EEPROM
                                Write(Settings::getSteps() == value? "OK" : "ERR");
                            }
                            else
                                Write("ERR");
                        }
                    }
                    else if (command.startsWith("ACC"))
                    {
                        command.replace("ACC ", "");
                        uint16_t value = command.toInt();
                        if (Settings::checkAcceleration(value))
                        {
                            Settings::setAcceleration(value);
                            // Check whether the value was successfully stored in EEPROM
                            Write(Settings::getAcceleration() == value? "OK" : "ERR");
                        }
                        else
                            Write("ERR");
                    }
                    else if (command.startsWith("EXP"))
                    {
                        command.replace("EXP ", "");
                        uint16_t value = command.toInt();
                        if (Settings::checkExposure(value))
                        {
                            Settings::setExposure(value);
                            // Check whether the value was successfully stored in EEPROM
                            Write(Settings::getExposure() == value? "OK" : "ERR");
                        }
                        else
                            Write("ERR");
                    }
                    else if (command.startsWith("DELAY"))
                    {
                        command.replace("DELAY ", "");
                        uint16_t value = command.toInt();
                        if (Settings::checkDelay(value))
                        {
                            Settings::setDelay(value);
                            // Check whether the value was successfully stored in EEPROM
                            Write(Settings::getDelay() == value? "OK" : "ERR");
                        }
                        else
                            Write("ERR");
                    }
                }
                else if (command == RunFreeMovement)
                {
                    Write("OK");
                    Runner::run(Runner::FreeMovement);
                }
                else if (command.startsWith("FM "))
                {
                    if (!Runner::isBusy() || Runner::getMode() != Runner::FreeMovement)
                    {
                        Write("ERR");
                    }
                    else
                    {
                        command.replace("FM ", "");
                        int value = command.toInt();
                        Runner::rotate(value * DEGREE);
                        Write("OK");
                    }
                }
                else if (command == IsRunning)
                {
                }
                else if (command == RunAutoMode)
                {
                    Write("OK");
                    Runner::run(Runner::Auto);
                }
                else if (command == RunManualMode)
                {
                }
                else if (command == RunNonStopMode)
                {
                }
                else if (command == RunVideoMode)
                {
                    Write("OK");
                    Runner::run(Runner::Video);
                }
                else if (command == RunRotateMode)
                {
                }
                else if (command == Shutter)
                {
                }
                else if (command == Next)
                {
                }
                else if (command == Stop)
                {
                    Runner::stop();
                    Write("OK");
                }
                else if (command == IncreasePWM)
                {
                    //TODO: analyze current mode and isRunning
                    Runner::setChangingPWM(1);
                    Write("OK");
                }
                else if (command == DecreasePWM)
                {
                    //TODO: analyze current mode and isRunning
                    Runner::setChangingPWM(-1);
                    Write("OK");
                }
                else
                {
                    Write(Undefined);
                    Write(command); //TODO temporary
                }
            }
        }
};
Listener listener;

void setup()
{
    pinMode(MOTOR_ENC1, INPUT);
    pinMode(MOTOR_ENC2, INPUT);
    pinMode(MOTOR1, OUTPUT);
    pinMode(MOTOR2, OUTPUT);
    pinMode(CAMERA, OUTPUT);
    pinMode(SHUTTER, OUTPUT);
    digitalWrite(SHUTTER, CAMERA_LOW); // release shutter
    digitalWrite(CAMERA, CAMERA_LOW); // release camera
    
    SetPinFrequencySafe(MOTOR1, 15000);
    SetPinFrequencySafe(MOTOR2, 15000);

    Serial.begin(115200);
    Serial.setTimeout(10);
}

void loop()
{
    mover.tick();
    Runner::tick();
    listener.tick();
}
