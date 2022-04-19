#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

#define MOTOR 9
#define DIRECTION 10
#define MOTOR_POWER 4
#define MOTOR_ENC1 2
#define MOTOR_ENC2 3
#define GRADUATIONS 4320 // number of graduations per turn
#define DEGREE (GRADUATIONS / 360)

int MIN_PWM = 50;
int MAX_PWM = 255;

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
            return 10;
        }

        // Returns number of graduations for acceleration and deceleration from min to max PWM and vice versa.
        // Function converts current user-friendly value 1-10 to this value.
        static uint16_t getRealAcceleration()
        {
            // Change these values if table does not work as expected:
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

        virtual void move(int32_t graduations, int maxPWM = MAX_PWM)
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
            _startDelay = 100;
            _started = false;
            _state = Move;

            analogWrite(MOTOR, _currentPWM);
            digitalWrite(DIRECTION, _forward? LOW : HIGH );
        }

        virtual void run(int pwm)
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
            _startDelay = 100;
            _started = false;
            _state = RunAcc;

            analogWrite(MOTOR, _currentPWM);
            digitalWrite(DIRECTION, _forward? LOW : HIGH );
        }

        virtual void stop()
        {
            analogWrite(MOTOR, 0);
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
            analogWrite(MOTOR, _currentPWM);
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

    protected:
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
        uint16_t _startDelay;
        bool _started;
        unsigned char _timePartCount;
        int _acceleration;
        int _realAcceleration;
        
        virtual bool start()
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

            if (millis() - _startTimer2 >= _startDelay)
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

                _startDelay = _startDelay / 1.5;
                _startTimer2 = millis();
                _minPWM += 5;
                if (_minPWM > limit)
                    _minPWM = limit;
                _currentPWM = _minPWM;
                analogWrite(MOTOR, _currentPWM);
                Serial.println("Increase PWM. New value: " + String(_minPWM));
            }

            return false;
        }

        int calcHighLimitOfMinPWM()
        {
            const int highestLimit = 100;
            
            switch (_state)
            {
                case RunAcc:
                    // In this case we can return a large enough value
                    return highestLimit;

                case Move:
                case Correction:
                    // Calculate maximum possible value
                    {
                        float halfPoint = _graduations / 2.0;
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
        
        virtual void tickMove()
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

                analogWrite(MOTOR, _currentPWM);
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
            float currentPWM = MIN_PWM + x * (MAX_PWM - MIN_PWM) / decelerationLength;
            _currentPWM = validatePWM(currentPWM);
        }

        virtual void makeCorrection()
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
                    analogWrite(MOTOR, _currentPWM);
                    break;
                    
                case RunDec:
                    decelerate();
                    if (_currentPWM <= MIN_PWM)
                        stop();
                    else
                        analogWrite(MOTOR, _currentPWM);
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

class MeasureMover : public Mover
{
    public:
        inline int getUnderError()
        {
            return _underError;
        }
        
        inline int getOverError()
        {
            return _overError;
        }

        inline bool started()
        {
            return _started;
        }

        inline bool canStart()
        {
            return _canStart;
        }

        inline bool canMove()
        {
            return _canMove;
        }

        inline float getLastMoveTime()
        {
            return (_stop - _start) / 1000.0;
        }

        void setVerifyMove(bool verifyMove)
        {
            _verifyMove = verifyMove;
        }

        virtual void move(int32_t graduations, int maxPWM = MAX_PWM)
        {
            _underError = 0;
            _overError = 0;
            _canStart = true;
            _canMove = true;
            _start = _timer2 = millis();
            Mover::move(graduations, maxPWM);
        }

        virtual void run(int pwm)
        {
            _underError = 0;
            _overError = 0;
            _canStart = true;
            _canMove = true;
            Mover::run(pwm);
        }

        virtual void stop()
        {
            _stop = millis();
            Mover::stop();
        }
        
    private:
        bool _verifyMove;
        unsigned long _timer2;
        int32_t _oldPos;
        unsigned long _start;
        unsigned long _stop;
        int _underError;
        int _overError;
        bool _canStart;
        bool _canMove;
        
        virtual bool start()
        {
            if (!canStart())
            {
                return false;
            }
               
            if (_started)
            {
                // Everything is OK, table started
                return true;
            }
           
            if (_currentPos != 0)
            {
                // Everything is OK, table started
                _oldPos = _currentPos;
                _started = true;
                return true;
            }

            if (millis() - _startTimer2 >= _startDelay)
            {
                _canStart = false;
                stop();
            }
            
            return false;
        }

        virtual void makeCorrection()
        {
            int error = _graduations - _currentPos;
            if (error < 0)
                _overError = -error;
            else if (error > 0)
                _underError = error;

            stop();
        }

        virtual void tickMove()
        {
            Mover::tickMove();
            if (!_verifyMove)
                return;

            if (_oldPos != _currentPos)
            {
                _oldPos = _currentPos;
                _timer2 = millis();
            }
            else if (millis() - _timer2 >= 1000)
            {
                _canMove = false;
                stop();
            }
        }
};
MeasureMover mover;

class MinMaxQualifier
{
    public:
        inline bool isDone()
        {
            return _isMinDetermined && _isMaxDetermined;
        }

        void start()
        {
            Serial.println("Deterime MIN_PWM...");
            _stage = DefineMin;
            _state = Stopped;
            _isMinDetermined = false;
            _isMaxDetermined = false;
            mover.setVerifyMove(true);
        }

        void tick()
        {
            if (isDone())
                return;

            if (!_isMinDetermined)
                DetermineMin();
            else
                DetermineMax();
        }

    private:
        enum State
        {
            Move,
            Stopped
        };

        enum Stage
        {
            DefineMin,
            DefineMax
        };

        State _state;
        Stage _stage;
        bool _isMinDetermined;
        bool _isMaxDetermined;
        int _low;
        int _high;
        int _lastGoodValue;
        
        void DetermineMin()
        {
            if (_state == Stopped)
            {
                Serial.print(String(MIN_PWM) + ": ");
                _state = Move;
                mover.move(10, MIN_PWM);
            }
            else if (mover.isStopped())
            {
                _state = Stopped;
                if (!mover.canStart() || !mover.canMove())
                {
                    Serial.println(mover.canStart() ? "Unable to move" : "Unable to start");
                    MIN_PWM++;
                    if (MIN_PWM > 255)
                    {
                        Serial.println();
                        Serial.println("Something goes wrong:-(");
                        while (true);
                    }
                    
                    Serial.print(String(MIN_PWM) + ": ");
                    _state = Move;
                    mover.move(10, MIN_PWM);
                }
                else
                {
                    _isMinDetermined = true;
                    Serial.println("MIN_PWM found!");
                }
            }
        }

        void DetermineMax()
        {
            if (_stage == DefineMin)
            {
                _stage = DefineMax;
                _state = Stopped;
                _low = MIN_PWM;
                _high = MAX_PWM;
                _lastGoodValue = -1;
                mover.setVerifyMove(false);
                Serial.println("______________________________________________________");
                Serial.println("Deterime MAX_PWM...");
            }
            
            if (_state == Stopped)
            {
                Serial.print(String(MAX_PWM) + ": ");
                _state = Move;
                mover.move(GRADUATIONS, MAX_PWM);
            }
            else if (mover.isStopped())
            {
                _state = Stopped;
                float time = mover.getLastMoveTime();
                if (mover.getOverError() == 0)
                {
                    _lastGoodValue = MAX_PWM;

                    // continue
                    Serial.println(String(time) + " sec, overError = " +
                        String(mover.getOverError()) + ", continue");

                    _low = MAX_PWM;
                    int middle = _low + (_high - _low) / 2;
                    MAX_PWM = middle;
                }
                else
                {
                    Serial.println("Too fast: " + String(time) + " sec, overError = " +
                        String(mover.getOverError()));
                    
                    _high = MAX_PWM;
                    int middle = _low + (_high - _low) / 2;
                    MAX_PWM = middle;
                }

                if (_high - _low <= 1)
                {
                    if (_lastGoodValue < 0)
                    {
                        Serial.println();
                        Serial.println("Something goes wrong:-(");
                        while (true);
                    }

                    MAX_PWM = _lastGoodValue;
                    Serial.println();
                    Serial.println("MAX_PWM found!");
                    Serial.println("______________________________________________________");
                    Serial.println("#define MIN_PWM " + String(MIN_PWM));
                    Serial.println("#define MAX_PWM " + String(MAX_PWM));
                    _isMaxDetermined = true;
                    return;
                }
            }
        }
};
MinMaxQualifier qualifier;

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
                    if (!mover.isStopped())
                        return;
    
                    _stop = millis();
                    _state = Measured;
            }
        }
    
        void measure(int pwm)
        {
            _pwm = pwm;
            _state = Measuring;
            _start = millis();
            mover.move(GRADUATIONS, _pwm);
        }
    
        String getOutput()
        {
            float time = (_stop - _start) / 1000.0;
            return String(_pwm) + "\t" + String(time);
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
        int _pwm;
};

class Measurer
{
    public:
        const int delta = 1;
        
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
                        Serial.println(_measurer.getOutput());
                        _pwm += delta;
                        if (_pwm > MAX_PWM)
                        {
                            _state = Measured;
                            return;
                        }
                        else
                        {
                            _measurer.measure(_pwm);
                        }
                    }
                    return;

                default:
                    return;
            }
        }

        void measure()
        {
            _pwm = MIN_PWM;
            _state = Measuring;
            _measurer.measure(_pwm);
        }
    
        void cancel()
        {
            _measurer.cancel();
            _state = Waiting;
        }
    
    private:
        State _state = Waiting;
        TurnMeasurer _measurer;
        int _pwm;
};

class Worker
{
    public:
        void tick()
        {
            _measurer.tick();
        }

        void start()
        {
            _measurer.measure();
        }

    private:
        Measurer _measurer;
};
Worker worker;

void setup()
{
    pinMode(MOTOR_ENC1, INPUT);
    pinMode(MOTOR_ENC2, INPUT);
    pinMode(MOTOR, OUTPUT);
    pinMode(DIRECTION, OUTPUT);
    pinMode(MOTOR_POWER, OUTPUT);
    analogWrite(MOTOR, 0);
    digitalWrite(MOTOR_POWER, HIGH);

    Serial.begin(9600);

    qualifier.start();
}

void loop()
{
    mover.tick();
    qualifier.tick();
    
//    worker.tick();
}
