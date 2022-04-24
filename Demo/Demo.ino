#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wsign-compare"
#include <EncButton.h>
#pragma GCC diagnostic pop

//#define RUSSIAN // uncomment to use Russian

#ifdef RUSSIAN
#define _LCD_TYPE 1
#include <LCD_1602_RUS.h>
#else
#include <LiquidCrystal_I2C.h>
#endif

#include <PWM.h>

#define MOTOR1 10
#define MOTOR2 9
#define MOTOR_ENC1 2
#define MOTOR_ENC2 3
#define MIN_PWM 60
#define MAX_PWM 255
#define GRADUATIONS 4320 // number of graduations per turn
#define DEGREE (GRADUATIONS / 360)

// Uncomment below to enable debug output.
//#define DEBUG_MODE

Encoder encoder(MOTOR_ENC1, MOTOR_ENC2);

#ifdef RUSSIAN
LCD_1602_RUS lcd(0x27, 16, 2);
#else
LiquidCrystal_I2C lcd(0x27, 16, 2);
#endif

EncButton<EB_TICK, 12, 13, 11> enc; // pins 11, 12, 13
EncButton<EB_TICK, 8> startButton;  // pin 8

bool startPressed()
{
    bool encPressed = enc.press();
    bool startBtnPressed = startButton.press();
    return encPressed || startBtnPressed;    
}

void writeMotorPWM(unsigned char pwm, bool forward = true)
{
    if (pwm == 0)
    {
        analogWrite(MOTOR1, 0);
        analogWrite(MOTOR2, 0);
    }
    else
        analogWrite(forward? MOTOR1 : MOTOR2, pwm);
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

class Displayer
{
    public:
        static void display(String top, String bottom)
        {
            printTop(top);
            printBottom(bottom);
        }

        static void printTop(String text)
        {
            if (_recentTop != text)
            {
                _recentTop = text;
                text = fillWithSpaces(text);
                lcd.setCursor(0, 0);
                lcd.print(text);
            }
        }

        static void printBottom(String text)
        {
            if (_recentBottom != text)
            {
                _recentBottom = text;
                text = fillWithSpaces(text);
                lcd.setCursor(0, 1);
                lcd.print(text);
            }
        }

    private:
        static String _recentTop;
        static String _recentBottom;

        static String fillWithSpaces(String text)
        {
            String result;
            result.reserve(16);
            result = text;
            while (result.length() < 16)
                result += " ";

            return result;
        }
};
String Displayer::_recentTop;
String Displayer::_recentBottom;

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
            _cumulativePos -= encoder.readAndReset();
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
            _cumulativePos -= encoder.readAndReset();
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
#ifdef DEBUG_MODE
                Serial.println("High limit of _minPWM: " + String(limit));
#endif
                if (_minPWM >= limit)
                {
                    // Unable to increase _minPWM anymore
                    // If correction, wait a second, and if table is still not moving, stop it
                    if (_state == Correction && millis() - _startTimer >= 1000)
                    {
#ifdef DEBUG_MODE
                        Serial.println("Unable to start, stopping...");
#endif
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
#ifdef DEBUG_MODE
                Serial.println("Increase PWM. New value: " + String(_minPWM));
#endif
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
#ifdef DEBUG_MODE
                Serial.println("Error = " + String(error) + ", correction...");
#endif
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
                    if (_currentPWM <= MIN_PWM)
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

class Handler
{
    public:
        virtual void start() = 0;

        void stop()
        {
            mover.stop();
            _isFinished = true;
        }

        virtual void tick() = 0;
        
        inline bool isFinished()
        {
            return _isFinished;
        }

    protected:
        bool _isFinished = false;
};

class RunHandler : public Handler
{
    public:
        virtual void start()
        {
            _isFinished = false;
            _isStopping = false;
            _seconds = random(4, 9);
            int direction = random(0, 2);
            if (direction == 0)
                direction = -1;
            _pwm = random(MIN_PWM, MAX_PWM + 1) * direction;
            Settings::setAcceleration(random(1, 11));

#ifdef RUSSIAN
            Displayer::display("Функция Run", String(_seconds) + "с");
#else
            Displayer::display("Run", String(_seconds) + "s");
#endif

            mover.run(_pwm);
            _timer = millis();
        }

        virtual void tick()
        {
            if (_isFinished)
                return;
                
            if (_isStopping && !mover.isStopped())
                return;

            if (mover.isStopped())
            {
                _isFinished = true;
                return;
            }

            if (millis() - _timer >= _seconds * 1000)
            {
                mover.softStop();
                _isStopping = true;
            }
        }

    private:
        unsigned long _timer;
        unsigned char _seconds;
        int _pwm;
        bool _isStopping;
};
RunHandler runHandler;

class MoveHandler : public Handler
{
    public:
        virtual void start()
        {
            _isFinished = false;

            _pwm = random(MIN_PWM, MAX_PWM + 1);
            _direction = random(0, 2);
            if (_direction == 0)
                _direction = -1;

            _graduations = random(10, 30) * _direction * 10;
            
            int acc = random(6, 11);
            Settings::setAcceleration(acc);

            _count = random(4, 9);
            _current = 0;

#ifdef RUSSIAN
            Displayer::display("Функция Move", String(_graduations));
#else
            Displayer::display("Move", String(_graduations));
#endif

            mover.move(_graduations, _pwm);
        }
        
        virtual void tick()
        {
            if (_isFinished)
                return;

            if (mover.isStopped())
            {
                _current++;
                if (_current >= _count)
                    _isFinished = true;
                else
                {
                    _graduations += _direction > 0 ? 100 : -100;
                    Displayer::printBottom(String(_graduations));
                    mover.move(_graduations, _pwm);
                }
            }
        }

    private:
        int _count;
        int _current;
        int _pwm;
        int _graduations;
        int _direction;
};
MoveHandler moveHandler;

class AccHandler : public Handler
{
    public:
        virtual void start()
        {
            _isFinished = false;
            _offset = random(0, 2);
            if (_offset == 0)
                _offset = -1;

            _current = _offset > 0 ? 1 : 10;

            int direction = random(0, 2);
            if (direction == 0)
                direction = -1;
                
            _distance = random(70, 300) * direction;

            Settings::setAcceleration(_current);

#ifdef RUSSIAN
            Displayer::display("Ускорение", String(_current));
#else
            Displayer::display("Acceleration", String(_current));
#endif
            
            mover.move(_distance);
        }
        
        virtual void tick()
        {
            if (_isFinished)
                return;

            if (mover.isStopped())
            {
                _current += _offset;
                if (_current < 1 || _current > 10)
                    _isFinished = true;
                else
                {
                    Settings::setAcceleration(_current);
                    Displayer::printBottom(String(_current));
                    mover.move(_distance);
                }
            }
        }

   private:
        int _offset;
        int _current;
        int _distance;
};
AccHandler accHandler;

class Worker
{
    public:
        Worker()
        {
            _current = random(0, 2);
            _isRunning = false;
        }

        void tick()
        {
            if (startPressed())
            {
                if (_isRunning)
                {
                    _handlers[_current]->stop();
                    _isRunning = false;
                    _isPaused = false;
                    Displayer::display("", "");
                }
                else
                {
                    _handlers[_current]->start();
                    _isRunning = true;
                    _isPaused = false;
                }
            }

            if (!_isRunning)
                return;

            if (_isPaused)
            {
                if (millis() - _timer < 800)
                    return;

                _handlers[_current]->start();
                _isPaused = false;
            }

            if (_handlers[_current]->isFinished())
            {
                _current = random(0, 3);
                _timer = millis();
                _isPaused = true;
                Displayer::display("", "");
                return;
            }

            _handlers[_current]->tick();
        }

    private:
        static const int _count = 3;
        Handler* _handlers[_count] = { &runHandler, &moveHandler, &accHandler };
        int _current;
        bool _isRunning;
        unsigned long _timer;
        bool _isPaused;
};
Worker worker;

void setup()
{
    pinMode(MOTOR1, OUTPUT);
    pinMode(MOTOR2, OUTPUT);
    lcd.init();
    lcd.begin(16, 2);
    lcd.clear();
    lcd.backlight();

    enc.setButtonLevel(HIGH);
    startButton.setButtonLevel(HIGH);
    SetPinFrequencySafe(MOTOR1, 15000);
    SetPinFrequencySafe(MOTOR2, 15000);
}

void loop()
{
    enc.tick();
    startButton.tick();
    mover.tick();
    worker.tick();
}
