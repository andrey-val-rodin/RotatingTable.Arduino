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

uint16_t EEMEM stepsOffset;
unsigned char EEMEM accelerationOffset;
uint16_t EEMEM delayOffset;
uint16_t EEMEM exposureOffset;
uint16_t EEMEM videoPWMOffset;
float EEMEM nonstopFrequencyOffset;

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

const char terminator = '\n';
void Write(const char* text)
{
    char strBuf[64];
    sprintf(strBuf, "%s%c", text, terminator);
    Serial.write(strBuf, strlen(strBuf));
}

void Write(const char* text, int arg)
{
    char strBuf[64];
    sprintf(strBuf, "%s%d%c", text, arg, terminator);
    Serial.write(strBuf);
}

void Write(const String& text)
{
    Write(text.c_str());
}

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
            if (!canChangePWM(delta))
                return;
                
            _maxPWM += delta;
            _maxPWM = PWMValidator::validate(_maxPWM);
            _currentPWM = _maxPWM;
            analogWrite(_forward? MOTOR1 : MOTOR2, _currentPWM);
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
class Runner
{
    public:
        const int delta = 5; // value to increment/decrement pwm

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

        enum Stopping : char
        {
            NoStopping = 0,
            Stop = 1,
            SoftStop = 2
        };

        void runAutomatic()
        {
            const String mode = "Auto...";
            const String stepName = "photo";
#ifdef DEBUG_MODE
            static int total = 0;
#endif
            
            if (getStopping())
            {
                finalize();
                return;
            }

            if (!mover.isStopped())
            {
                if (millis() - _timer2 >= 50)
                {
                    _currentAngle = mover.getLatestPos() / DEGREE;
                    if (_currentAngle != _oldAngle)
                    {
                        _oldAngle = _currentAngle;
                        Write("POS ", _currentAngle);
                    }

                    _timer2 = millis();
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
                _timer = _timer2 = millis();
                _currentState = Beginning;
#ifdef DEBUG_MODE
                total = 0;
#endif
                return;
            }

            if (_currentState == Beginning && millis() - _timer >= 10) // 10 ms is a time for preparing camera
            {
                _stepNumber++;
                Write("STEP ", _stepNumber);
                digitalWrite(SHUTTER, CAMERA_HIGH); // make first photo
                _timer = _timer2 = millis();
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
                    _timer = _timer2 = millis();
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

            if (_currentState == Delay)
            {
                if (millis() - _timer2 >= 1000)
                {
                    // Send token at least once per second
                    Write("WAIT");
                    _timer2 = millis();
                }

                if (millis() - _timer >= Settings::getDelay())
                {
                    _stepNumber++;
                    Write("STEP ", _stepNumber);
                    digitalWrite(SHUTTER, CAMERA_HIGH); // make photo
                    _timer = millis();
                    _currentState = Exposure;
                }
            }
        }

        void runManual()
        {
            const String mode = "Manual...";
            const String stepName = "step";
#ifdef DEBUG_MODE
            static int total = 0;
#endif
            
            if (getStopping())
            {
                finalize();
                return;
            }

            if (!mover.isStopped())
            {
                if (millis() - _timer2 >= 50)
                {
                    _currentAngle = mover.getLatestPos() / DEGREE;
                    if (_currentAngle != _oldAngle)
                    {
                        _oldAngle = _currentAngle;
                        Write("POS ", _currentAngle);
                    }

                    _timer2 = millis();
                }

                return;
            }

            int16_t stepCount = Settings::getSteps();
            int stepGraduations = GRADUATIONS / stepCount;
            if (!_isBusy)
            {
                // Starting
                _stepNumber = 1;
                Write("STEP ", _stepNumber);
                _needToMove = false;
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

            if (isPhoto() && _currentState == Waiting)
            {
                digitalWrite(SHUTTER, CAMERA_HIGH); // make photo
                _timer = millis();
                _currentState = Exposure;
                setPhoto(false);
                return;
            }

            if (isNext() && _currentState == Waiting)
            {
                digitalWrite(SHUTTER, CAMERA_HIGH); // make photo
                _timer = millis();
                _currentState = Exposure;
                _needToMove = true;
                setNext(false);
                return;
            }

            if (_currentState == Move && mover.isStopped())
            {
                if (_stepNumber < stepCount)
                {
                    _currentState = Waiting;
                    _stepNumber++;
                    Write("STEP ", _stepNumber);
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

        void runNonstop()
        {
            const String mode = "Nonstop...";
            const String stepName = "photo";
#ifdef DEBUG_MODE
            static int total = 0;
#endif

            if (getStopping())
            {
                finalize();
                return;
            }
            else if (!mover.isStopped())
            {
                if (millis() - _timer2 >= 50)
                {
                    _currentAngle = mover.getLatestPos() / DEGREE;
                    if (_currentAngle != _oldAngle)
                    {
                        _oldAngle = _currentAngle;
                        Write("POS ", _currentAngle);
                    }

                    _timer2 = millis();
                }

                if (isChangingPWM())
                {
                    if (mover.isUniformMotion())
                    {
                        if (isDecreasePWM())
                        {
                            int d = calcNonstopDelta(true);
                            if (canChangePWM(d))
                            {
                                mover.changePWM(d);
                                _needToStoreNewPWM = true;
                            }
                        }
                        else if (isIncreasePWM())
                        {
                            int d = calcNonstopDelta(false);
                            if (canChangePWM(d))
                            {
                                mover.changePWM(d);
                                _needToStoreNewPWM = true;
                            }
                        }
                    }

                    setChangingPWM(0);
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
                _timer = _timer2 = millis();
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
                Write("STEP ", _stepNumber);
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
                    Write("STEP ", _stepNumber);
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
        }

        void runVideo()
        {
            if (!_isRunning)
            {
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
            if (getStopping())
            {
                if (mover.isUniformMotion() && getStopping() == SoftStop)
                {
                    Settings::setVideoPWM(mover.getMaxPWM() * direction);
                    mover.softStop();
                }
                else
                {
                    mover.stop();
                }

                setStopping(NoStopping); // reset flag
            }
            else
            {
                if (millis() - _timer >= 50)
                {
                    _currentAngle = mover.getAbsolutePos() / DEGREE;
                    if (_currentAngle != _oldAngle)
                    {
                        _oldAngle = _currentAngle;
                        Write("POS ", _currentAngle);
                    }

                    _timer = millis();
                }
                
                if (isChangingPWM())
                {
                    if (mover.isUniformMotion())
                    {
                        if (isDecreasePWM())
                        {
                            if (needToChangeDirection(true))
                            {
                                // Change direction
                                mover.stop();
                                mover.run(-MIN_PWM);
                                return;
                            }

                            int d = -delta * direction;
                            if (canChangePWM(d))
                                mover.changePWM(d);
                        }
                        else if (isIncreasePWM())
                        {
                            if (needToChangeDirection(false))
                            {
                                // Change direction
                                mover.stop();
                                mover.run(MIN_PWM);
                                return;
                            }
                
                            int d = delta * direction;
                            if (canChangePWM(d))
                                mover.changePWM(delta * direction);
                        }
                    }

                    setChangingPWM(0);
                }
            }
        }

        void runFreeMovement()
        {
            if (!_isBusy)
            {
                _currentState = Waiting;
                _timer2 = millis();
                _isBusy = true;
            }
            
            if (getStopping())
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
                if (millis() - _timer2 >= 50)
                {
                    _currentAngle = mover.getLatestPos() / DEGREE;
                    if (_currentAngle != _oldAngle)
                    {
                        _oldAngle = _currentAngle;
                        Write("POS ", _currentAngle);
                    }

                    _timer2 = millis();
                }
            }
        }

        inline bool isRunning()
        {
            return _isRunning;
        }

        inline bool isBusy()
        {
            return _isBusy;
        }

        inline Mode getMode()
        {
            return _mode;
        }

        void run(Mode mode)
        {
            _mode = mode;
        }

        void rotate(int pos)
        {
            _currentState = Beginning;
            mover.move(pos);
        }

        void tick()
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

        void setStopping(Stopping stopping)
        {
            _stop = stopping;
        }

        void setChangingPWM(int value)
        {
            _changePWM = value;
        }

        bool canIncreasePWM()
        {
            char direction = mover.isForward() ? 1 : -1;
            int d;
            switch (getMode())
            {
                case Video:
                    d = delta * direction;
                    return needToChangeDirection(false) ? true : canChangePWM(d);

                case Nonstop:
                    d = calcNonstopDelta(false);
                    return mover.getCurrentPWM() < Settings::getHighRealNonstopPWM() && canChangePWM(d);

                default:
                    return false;
            }
        }

        bool canDecreasePWM()
        {
            char direction = mover.isForward() ? 1 : -1;
            int d;
            switch (getMode())
            {
                case Video:
                    d = -delta * direction;
                    return needToChangeDirection(true) ? true : canChangePWM(d);

                case Nonstop:
                    d = calcNonstopDelta(true);
                    return mover.getCurrentPWM() > Settings::getLowRealNonstopPWM() && canChangePWM(d);
                default:
                    return false;
            }
        }

        inline bool isUniformMotion()
        {
            return mover.isUniformMotion();
        }

        inline bool isNext()
        {
            return _next;
        }

        void setNext(bool value)
        {
            _next = value;
        }

        inline bool isPhoto()
        {
            return _photo;
        }

        void setPhoto(bool value)
        {
            _photo = value;
        }

    private:
        enum State : char
        {
            Waiting,
            Beginning,
            Delay,
            Exposure,
            Move
        };

        Mode _mode = None;
        int16_t _stepNumber = 0;
        unsigned long _timer = 0;
        unsigned long _timer2 = 0;
        bool _isRunning = false;
        bool _isBusy = false;
        Stopping _stop = NoStopping;
        State _currentState;
        int32_t _lastGraduations;
        bool _needToMove;
        int _nextSnapshotPos;
        bool _needToStoreNewPWM;
        int _currentAngle;
        int _oldAngle;
        int _changePWM = 0;
        bool _next = false;
        bool _photo = false;
    
        void finalize()
        {
            _isRunning = _isBusy = false;
            _stop = NoStopping;
            _mode = None;
            mover.stop();
            _stepNumber = 0;
            _currentAngle = _oldAngle = 0;
            _changePWM = 0;
             _next = _photo = false;
            digitalWrite(SHUTTER, CAMERA_LOW); // release shutter
            digitalWrite(CAMERA, CAMERA_LOW); // release camera
            Write("END");
        }

        inline Stopping getStopping()
        {
            return _stop;
        }

        inline bool isIncreasePWM()
        {
            return _changePWM > 0;
        }

        inline bool isDecreasePWM()
        {
            return _changePWM < 0;
        }

        bool canChangePWM(int delta)
        {
            if (!isRunning())
                return false;

            Mode mode = getMode();
            if (mode != Video && mode != Nonstop)
                return false;

            if (!mover.isUniformMotion())
            {
                // Assume that changing willbe available after acceleration
                return true;
            }

            return mover.canChangePWM(delta);
        }

        inline bool isChangingPWM()
        {
            return _changePWM != 0;
        }

        bool needToChangeDirection(bool decreasing)
        {
            char direction = mover.isForward() ? 1 : -1;
            if (decreasing)
            {
                if (direction > 0 && mover.getCurrentPWM() <= MIN_PWM)
                    return true;
            }
            else
            {
                if (direction < 0 && mover.getCurrentPWM() <= MIN_PWM)
                    return true;
            }

            return false;
        }

        int calcNonstopDelta(bool decreasing)
        {
            int res;
            if (decreasing)
            {
                res = delta;
                if (mover.getCurrentPWM() - res < Settings::getLowRealNonstopPWM())
                    res = mover.getCurrentPWM() - Settings::getLowRealNonstopPWM();
                res = -res;
            }
            else
            {
                res = delta;
                if (mover.getCurrentPWM() + res > Settings::getHighRealNonstopPWM())
                    res = Settings::getHighRealNonstopPWM() - mover.getCurrentPWM();
            }

            return res;
        }
};
Runner runner;

class Listener
{
    public:
/*    See https://github.com/andrey-val-rodin/RotatingTable.Xamarin/blob/master/RotatingTable.Xamarin/RotatingTable.Xamarin/Models/Commands.cs
        const String Status          = "STATUS";
        const String Ready           = "READY";
        const String Running         = "RUNNING";
        const String Busy            = "BUSY";
        const String End             = "END";
        const String OK              = "OK";
        const String Error           = "ERR";
        const String GetSteps        = "GET STEPS";
        const String Step            = "STEP ";
        const String GetAcceleration = "GET ACC";
        const String GetExposure     = "GET EXP";
        const String GetDelay        = "GET DELAY";
        const String SetAcceleration = "SET ACC";
        const String SetSteps        = "SET STEPS";
        const String SetExposure     = "SET EXP";
        const String SetDelay        = "SET DELAY";
        const String Position        = "POS ";
        const String GetMode         = "GET MODE";
        const String RunAutoMode     = "RUN AUTO";
        const String RunManualMode   = "RUN MANUAL";
        const String RunNonStopMode  = "RUN NS";
        const String RunVideoMode    = "RUN VIDEO";
        const String RunFreeMovement = "RUN FM";
        const String FreeMovement    = "FM ";
        const String Shutter         = "SHUTTER";
        const String Next            = "NEXT";
        const String Stop            = "STOP";
        const String SoftStop        = "SOFTSTOP";
        const String IncreasePWM     = "INCPWM";
        const String DecreasePWM     = "DECPWM";
        const String Undefined       = "UNDEF";
*/
        void tick()
        {
            if (Serial.available())
            {
                String command = Serial.readStringUntil(terminator);
                if (command == "STATUS")
                {
                    if (runner.isRunning())
                        Write("RUNNING");
                    else if (runner.isBusy())
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
                }
                else if (command.startsWith("SET "))
                {
                    command.replace("SET ", "");
                    if (command.startsWith("STEPS "))
                    {
                        if (runner.getMode() != runner.None)
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
                else if (command == "RUN AUTO")
                {
                    if (runner.isBusy() || runner.isRunning())
                        Write("ERR");
                    else
                    {
                        Write("OK");
                        runner.run(runner.Auto);
                    }
                }
                else if (command == "RUN MANUAL")
                {
                    if (runner.isBusy() || runner.isRunning())
                        Write("ERR");
                    else
                    {
                        Write("OK");
                        runner.run(runner.Manual);
                    }
                }
                else if (command == "RUN NS")
                {
                    if (runner.isBusy() || runner.isRunning())
                        Write("ERR");
                    else
                    {
                        Write("OK");
                        runner.run(runner.Nonstop);
                    }
                }
                else if (command == "RUN VIDEO")
                {
                    if (runner.isBusy() || runner.isRunning())
                        Write("ERR");
                    else
                    {
                        Write("OK");
                        runner.run(runner.Video);
                    }
                }
                else if (command == "RUN FM")
                {
                    if (runner.isBusy() || runner.isRunning())
                        Write("ERR");
                    else
                    {
                        Write("OK");
                        runner.run(runner.FreeMovement);
                    }
                }
                else if (command.startsWith("FM "))
                {
                    if (!runner.isBusy() || runner.getMode() != runner.FreeMovement)
                    {
                        Write("ERR");
                    }
                    else
                    {
                        Write("OK");
                        command.replace("FM ", "");
                        int value = command.toInt();
                        runner.rotate(value * DEGREE);
                    }
                }
                else if (command == "SHUTTER")
                {
                    if (!runner.isBusy() || runner.getMode() != runner.Manual)
                    {
                        Write("ERR");
                    }
                    else
                    {
                        Write("OK");
                        runner.setPhoto(true);
                    }
                }
                else if (command == "NEXT")
                {
                    if (!runner.isBusy() || runner.getMode() != runner.Manual)
                    {
                        Write("ERR");
                    }
                    else
                    {
                        Write("OK");
                        runner.setNext(true);
                    }
                }
                else if (command == "STOP")
                {
                    if (runner.isBusy() || runner.isRunning())
                    {
                        Write("OK");
                        runner.setStopping(runner.Stop);
                    }
                    else
                        Write("ERR");
                }
                else if (command == "SOFTSTOP")
                {
                    if (runner.getMode() == runner.Video && runner.isUniformMotion())
                    {
                        Write("OK");
                        runner.setStopping(runner.SoftStop);
                    }
                    else
                        Write("ERR");
                }
                else if (command == "INCPWM")
                {
                    if (runner.canIncreasePWM())
                    {
                        Write("OK");
                        runner.setChangingPWM(1);
                    }
                    else
                        Write("ERR");
                }
                else if (command == "DECPWM")
                {
                    if (runner.canDecreasePWM())
                    {
                        Write("OK");
                        runner.setChangingPWM(-1);
                    }
                    else
                        Write("ERR");
                }
                else
                {
                    Write("UNDEF:" + command);
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
    runner.tick();
    listener.tick();
}
