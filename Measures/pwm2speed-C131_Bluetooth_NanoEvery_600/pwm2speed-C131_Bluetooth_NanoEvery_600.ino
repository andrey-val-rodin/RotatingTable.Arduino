#include <Encoder.h>

#define MOTOR1 10
#define MOTOR2 9
#define MOTOR_ENC1 2
#define MOTOR_ENC2 3
#define MIN_PWM 48
#define MAX_PWM 255
#define GRADUATIONS 4320 // number of graduations per turn
#define DEGREE (GRADUATIONS / 360)

Encoder encoder(MOTOR_ENC1, MOTOR_ENC2);

unsigned int timeMultiplier = 16;
// TODO
void SetupHardwareTimer()
{
    // Turn off timer while we change parameters
    TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_ENABLE_bm;
    // Clear all CLKSEL bits
    TCA0.SINGLE.CTRLA &= ~TCA_SINGLE_CLKSEL_gm;
    // Set prescaler to 4
    TCA0.SINGLE.CTRLA |= TCA_SINGLE_CLKSEL_DIV4_gc;
    // // Re-enable timer
    TCA0.SINGLE.CTRLA |= TCA_SINGLE_ENABLE_bm;
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
            return 10;
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

        static int16_t getSteps()
        {
            return 2;
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
            _minPWM = MIN_PWM;
            _maxPWM = maxPWM;
            _cumulativePos -= encoder.readAndReset();
            _acceleration = Settings::getAcceleration();
            _realAcceleration = Settings::getRealAcceleration();
            _startTimer2 = _startTimer = millis();
            _started = false;
            _state = Move;

            analogWrite(_forward? MOTOR1 : MOTOR2, _currentPWM);
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

            analogWrite(_forward? MOTOR1 : MOTOR2, _currentPWM);
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
            if (millis() - _startTimer2 >= 100 * timeMultiplier)
            {
                int limit = calcHighLimitOfMinPWM();

                Serial.println("High limit of _minPWM: " + String(limit));
                
                if (_minPWM >= limit)
                {
                    // Unable to increase _minPWM anymore
                    // If correction, wait a second, and if table is still not moving, stop it
                    if (_state == Correction && millis() - _startTimer >= 1000 * timeMultiplier)
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
                analogWrite(_forward? MOTOR1 : MOTOR2, _currentPWM);

                Serial.println("Increase PWM. New value: " + String(_minPWM));
            }

            return false;
        }

        int calcHighLimitOfMinPWM()
        {
            int highestLimit = min(100, _maxPWM);
Serial.print(" highestLimit: "); Serial.print(highestLimit);
            
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
Serial.print("halfPoint: "); Serial.print(halfPoint);
                        float accelerationLength = _realAcceleration;
Serial.print(" accelerationLength: "); Serial.print(accelerationLength);
                        float value = MIN_PWM + halfPoint * (MAX_PWM - MIN_PWM) / accelerationLength;
Serial.print(" value: "); Serial.print(value);
Serial.print(" _maxPWM: "); Serial.print(_maxPWM);
                        int result = validatePWM(value);
Serial.print(" result after validate: "); Serial.print(result);
                        if (result > highestLimit)
                            result = highestLimit;
Serial.print(" result: "); Serial.println(result);

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

                analogWrite(_forward? MOTOR1 : MOTOR2, _currentPWM);
                return;
            }

            // Wait until table die
            if (_state == Stopping && millis() - _timer >= 10 * timeMultiplier)
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
                    analogWrite(_forward? MOTOR1 : MOTOR2, _currentPWM);
                    break;
                    
                case RunDec:
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
            else if (pwm < _minPWM)
                pwm = _minPWM;

            return pwm;
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

        float getTime()
        {
            return (_stop - _start) / 1000.0 / timeMultiplier;
        }
        
        String getOutput()
        {
            return String(_pwm) + "\t" + String(getTime());
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

const int delta = 1;
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
                        append(_measurer.getTime());
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

        void printTimes()
        {
            Serial.print("{ ");
            for (int i = 0; i < MAX_PWM - MIN_PWM + 1; i++)
            {
                if (i > 0)
                    Serial.print(", ");

                Serial.print(String(_times[i]));
            }
            Serial.print(" }");
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
        unsigned char _times[MAX_PWM - MIN_PWM + 1];

        void append(float time)
        {
            int index = _pwm - MIN_PWM;
            int value = round(time);
            if (value > 255)
                value = 255;

            _times[index] = (unsigned char)value;
        }
};

class Worker
{
    public:
        void start()
        {
            _measurer.measure();
        }
    
        void tick()
        {
            _measurer.tick();
            if (_measurer.getState() == Measured)
            {
                // finishing
                Serial.println();
                _measurer.printTimes();
                while (true);
            }
        }
        
    private:
        Measurer _measurer;
};
Worker worker;

// Set PWM repeat frequency for all PWM outputs with
// hardware support.
// The argument is the desired frequency in kHz. A
// best effort will be made to find something that matches.
void analogWriteFrequency(uint8_t kHz)
{
  static const byte index2setting[] = {
#if F_CPU > 1000000L
#if F_CPU > 2000000L
#if F_CPU > 4000000L
#if F_CPU > 8000000L
    TCA_SPLIT_ENABLE_bm | TCA_SPLIT_CLKSEL_DIV64_gc, // ~1 kHz PWM, ~250kHz clock
#endif
    TCA_SPLIT_ENABLE_bm | TCA_SPLIT_CLKSEL_DIV16_gc, // ~2 kHz is not possible, use 4
#endif
    TCA_SPLIT_ENABLE_bm | TCA_SPLIT_CLKSEL_DIV16_gc, // ~4 kHz PWM, ~1MHz clock
#endif
    TCA_SPLIT_ENABLE_bm | TCA_SPLIT_CLKSEL_DIV8_gc, // ~8 kHz PWM, ~2MHz clock
#endif
    TCA_SPLIT_ENABLE_bm | TCA_SPLIT_CLKSEL_DIV4_gc, // ~16 kHz PWM, ~4MHz clock
    TCA_SPLIT_ENABLE_bm | TCA_SPLIT_CLKSEL_DIV2_gc, // ~32 kHz PWM, ~8MHz clock
    TCA_SPLIT_ENABLE_bm | TCA_SPLIT_CLKSEL_DIV1_gc  // ~64 kHz PWM, ~16MHz clock
  };
  uint8_t index = 0;

  while (kHz > 1)
  { // find approximate match
    kHz >>= 1;
    if (++index >= sizeof(index2setting) - 1) break;
  }
  TCA0.SPLIT.CTRLA = index2setting[index];

  // note that this setting also influences Tone.cpp
}

void setup()
{
    pinMode(MOTOR1, OUTPUT);
    pinMode(MOTOR2, OUTPUT);

    SetupHardwareTimer();
//    analogWriteFrequency(16);

    Serial.begin(115200);

    worker.start();
}

void loop()
{
    mover.tick();
    worker.tick();
}