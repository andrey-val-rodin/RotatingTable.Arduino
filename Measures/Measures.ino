#include <Encoder.h>
#include <EncButton.h>
#include <PWM.h>

#define MOTOR1 10
#define MOTOR2 9
#define MOTOR_ENC1 2
#define MOTOR_ENC2 3
#define MAX_PWM 255
#define GRADUATIONS 4320 // number of graduations per turn
#define DEGREE (GRADUATIONS / 360)

int MIN_PWM = 10;
const int delta = 1;

Encoder encoder(MOTOR_ENC1, MOTOR_ENC2);

EncButton<EB_TICK, 12, 13, 11> enc; // pins 11, 12, 13

class SpeedValidator
{
    public:
        static int validate(int speed)
        {
            if (speed > MAX_PWM)
                speed = MAX_PWM;
            else if (speed < MIN_PWM)
                speed = MIN_PWM;

            return speed;
        }
};

class Settings
{
    public:
        static char getAcceleration()
        {
            return 10;
        }

        // Returns number of graduations for acceleration and deceleration from min to max PWM and vice versa.
        // Function converts current user-friendly value 1-10 to this native value.
        // Note that native value shouldn't be less than 80 when GRADUATIONS = 4320.
        static int getNativeAcceleration()
        {
            int result = getAcceleration();
            result = abs(result - 11); // reverse
            result *= 10;
            result += 10;
            result *= DEGREE;
            result /= 3;
            return result; // value in range from 80 to 440 when GRADUATIONS = 4320
        }
};

class Mover
{
    public:
        enum State : char
        {
            Stop,
            Move,
            RunAcc,
            Run,
            RunDec
        };

        void tick()
        {
            switch (_state)
            {
                case Move:
                    tickMove();
                    break;
                    
                case Run:
                case RunAcc:
                case RunDec:
                    tickRun();
                    break;
            }
        }

        State getState()
        {
            return _state;
        }

        void move(int graduations, int maxSpeed = MAX_PWM)
        {
            if (!isStopped())
                return;

            _graduations = abs(graduations);
            _forward = graduations > 0;
            _currentSpeed = MIN_PWM;
            _maxSpeed = maxSpeed;
            encoder.readAndReset();
            _state = Move;
        }

        void run(int speed)
        {
            if (!isStopped())
                return;

            _maxSpeed = abs(speed);
            _forward = speed > 0;
            _currentSpeed = MIN_PWM;
            encoder.readAndReset();
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
            if( _state == Run || _state == RunAcc || _state == RunDec)
            {
                // Calculate stop point
                float decelerationLength = Settings::getNativeAcceleration();
                float graduationsToStop = (_currentSpeed - MIN_PWM) * decelerationLength /
                    (MAX_PWM - MIN_PWM);
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
        inline int getCurrentSpeed()
        {
            return _currentSpeed;
        }

        // Returns maximum PWM
        inline int getMaxSpeed()
        {
            return _maxSpeed;
        }

        void changeSpeed(int delta)
        {
            switch (_state)
            {
                case Move:
                    if (_currentSpeed == _maxSpeed)
                    {
                        _maxSpeed += delta;
                        _maxSpeed = SpeedValidator::validate(_maxSpeed);
                        _currentSpeed = _maxSpeed;
                        analogWrite(_forward? MOTOR1 : MOTOR2, _currentSpeed);
                    }
                    break;
                    
                case Run:
                    _maxSpeed += delta;
                    _maxSpeed = SpeedValidator::validate(_maxSpeed);
                    _currentSpeed = _maxSpeed;
                    analogWrite(_forward? MOTOR1 : MOTOR2, _currentSpeed);
                    break;
            }
        }

        // Returns graduation count passed from starting point
        int getCurrentPos()
        {
            int32_t pos = abs(encoder.read());
            return pos;
        }

    private:
        State _state = Stop;
        int _graduations;
        bool _forward;
        int _maxSpeed = MAX_PWM;
        int _currentPos;
        int _currentSpeed;

        void tickMove()
        {
            _currentPos = getCurrentPos();
            if (_currentPos >= _graduations)
            {
                stop();
                return;
            }

            if (_currentPos < _graduations / 2)
                accelerate();
            else
                decelerate();

            analogWrite(_forward? MOTOR1 : MOTOR2, _currentSpeed);
        }

        void accelerate()
        {
            // Use linear function to accelerate
            float x = _currentPos;
            float accelerationLength = Settings::getNativeAcceleration();
            float currentSpeed = MIN_PWM + x * (MAX_PWM - MIN_PWM) / accelerationLength;
            _currentSpeed = validateSpeed(currentSpeed);
        }

        void decelerate()
        {
            // Use linear function to decelerate
            float x = _graduations - _currentPos - getFinalDistance();
            float decelerationLength = Settings::getNativeAcceleration();
            float currentSpeed = MIN_PWM + x * (MAX_PWM - MIN_PWM) / decelerationLength;
            _currentSpeed = validateSpeed(currentSpeed);
        }

        // End of the step we should go with MIN_PWM
        // This function returns length of this final distance
        int getFinalDistance()
        {
            switch (Settings::getAcceleration())
            {
                case 10:
                    return 16;
                case 9:
                    return 12;
                case 8:
                    return 6;
                case 7:
                    return 3;
                default:
                    return 0;
            }
        }
        
        void tickRun()
        {
            switch (_state)
            {
                case RunAcc:
                    accelerate();
                    analogWrite(_forward? MOTOR1 : MOTOR2, _currentSpeed);
                    if (_currentSpeed == _maxSpeed)
                        _state = Run;
                    break;
                    
                case RunDec:
                    decelerate();
                    if (_currentSpeed <= MIN_PWM)
                        stop();
                    else
                        analogWrite(_forward? MOTOR1 : MOTOR2, _currentSpeed);
                    break;
            }
        }

        int validateSpeed(int speed)
        {
            if (speed > _maxSpeed)
                speed = _maxSpeed;
            else if (speed < MIN_PWM)
                speed = MIN_PWM;

            return speed;
        }
};
Mover mover;

enum State
{
    Waiting,
    Measuring,
    Measured
};

class TurnMeasurer
{
    public:
        inline getState()
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
            char strBuf[40];
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
        inline getState()
        {
            return _state;
        }

        void tick()
        {
            _measurer.tick();
            
            switch (_state)
            {
                case Waiting:
                    return;

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
        inline int getStage()
        {
            return _stage;
        }
        
        void tick()
        {
            _measurer.tick();
            
            switch (_stage)
            {
                case 1:
                    if (_measurer.getState() == Measured)
                        setStage(2);
                    return;

                case 2:
                    _stage = 0;
            }
        }

        void start(int stage = 1)
        {
            setStage(stage);
        }

        void cancel()
        {
            _stage = 0;
            _measurer.cancel();
        }
        
    private:
        int _stage = 0;
        Measurer _measurer;

        void setStage(int stage)
        {
            switch (stage)
            {
                case 1:
                    Serial.println("Стандартная частота");
                    _stage = 1;
                    MIN_PWM = 10;
                    break;

                case 2:
                    Serial.println("15000 Гц");
                    SetPinFrequencySafe(MOTOR1, 15000);
                    SetPinFrequencySafe(MOTOR2, 15000);
                    MIN_PWM = 60;
                    break;

                default:
                    return;
            }
            
            _measurer.measure();
        }
};
Worker worker;

void setup()
{
    pinMode(MOTOR_ENC1, INPUT);
    pinMode(MOTOR_ENC2, INPUT);
    pinMode(MOTOR1, OUTPUT);
    pinMode(MOTOR2, OUTPUT);

    Serial.begin(9600);
}

void loop()
{
    enc.tick();
    mover.tick();
    worker.tick();

    if (enc.press())
    {
        if (worker.getStage() == 0)
            worker.start(2);
        else
            worker.cancel();
    }
}
