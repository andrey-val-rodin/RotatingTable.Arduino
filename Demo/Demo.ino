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

Encoder encoder(MOTOR_ENC1, MOTOR_ENC2);

#ifdef RUSSIAN
LCD_1602_RUS lcd(0x27, 16, 2);
#else
LiquidCrystal_I2C lcd(0x27, 16, 2);
#endif

EncButton<EB_TICK, 12, 13, 11> enc; // pins 11, 12, 13

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
            if (enc.press())
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
    pinMode(MOTOR_ENC1, INPUT);
    pinMode(MOTOR_ENC2, INPUT);
    pinMode(MOTOR1, OUTPUT);
    pinMode(MOTOR2, OUTPUT);
    lcd.init();
    lcd.begin(16, 2);
    lcd.clear();
    lcd.backlight();

    SetPinFrequencySafe(MOTOR1, 15000);
    SetPinFrequencySafe(MOTOR2, 15000);

    Serial.begin(9600);
}

void loop()
{
    enc.tick();
    mover.tick();
    worker.tick();
}
