#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <PWM.h>

#define MOTOR1 10
#define MOTOR2 9
#define MOTOR_ENC1 2
#define MOTOR_ENC2 3
#define MIN_PWM 60
#define MAX_PWM 255
#define GRADUATIONS 4320 // number of graduations per turn
#define DEGREE (GRADUATIONS / 360)

Encoder encoder(MOTOR_ENC1, MOTOR_ENC2);

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
        static unsigned char getAcceleration()
        {
            return validateAcceleration(_acceleration);
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

        static unsigned char validateAcceleration(unsigned char value)
        {
            return 1 <= value && value <= 10
                ? value
                : 7; // use default
        }

        static void setAcceleration(unsigned char value)
        {
            _acceleration = value;
        }
    
    private:
        static char _acceleration;
};
char Settings::_acceleration;

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
                float decelerationLength = Settings::getRealAcceleration();
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
            float accelerationLength = Settings::getRealAcceleration();
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
            float decelerationLength = Settings::getRealAcceleration();
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
                Serial.println("Error = " + String(error) + " Acceleration = " +
                    String(Settings::getAcceleration()) + " _maxPWM = " + String(_maxPWM));
                stop();
                move(error, MIN_PWM);
                _state = Correction;
            }
        }

        // We should pass end of step with MIN_PWM
        // This function returns length of this final distance
        int getFinalDistance()
        {
            switch (Settings::getAcceleration())
            {
                case 10:
                    return _graduations > 10 * DEGREE ? 16 : 12;
                case 9:
                    return _graduations > 10 * DEGREE ? 10 : 8;
                case 8:
                    return _graduations > 10 * DEGREE ? 6 : 4;
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

class Worker
{
    public:
        Worker()
        {
            _isPaused = false;
            _isMoved = false;
        }

        void start()
        {
            _isPaused = false;
        }

        void tick()
        {
            if (_isPaused)
            {
                if (millis() - _timer < 2000)
                    return;

                _isPaused = false;
            }

            if (_isMoved)
            {
                if (!mover.isStopped())
                    return;

                _isMoved = false;
                _isPaused = true;
                _timer = millis();
                return;
            }
            
            int pwm = random(100, 250);
            int distance = random(300, 8000);
            int direction = random(0, 2);
            if (direction == 0)
                direction = -1;
            distance *= direction;

            _isMoved = true;
            mover.move(distance, pwm);
        }

    private:
        unsigned long _timer;
        bool _isMoved;
        bool _isPaused;
};
Worker worker;

void setup()
{
    pinMode(MOTOR_ENC1, INPUT);
    pinMode(MOTOR_ENC2, INPUT);
    pinMode(MOTOR1, OUTPUT);
    pinMode(MOTOR2, OUTPUT);

    SetPinFrequencySafe(MOTOR1, 15000);
    SetPinFrequencySafe(MOTOR2, 15000);

    Settings::setAcceleration(1);
}

void loop()
{
    mover.tick();
    worker.tick();
}
