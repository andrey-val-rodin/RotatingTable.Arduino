# Скетч для поворотного стола
В начале программы находятся директивы препроцессора, определяющие используемые пины, минимальный и максимальный ШИМ-сигнал двигателя и количество меток энкодера для полного оборота стола (текущее значение - **4320**). Для диагностического вывода нужно раскомментировать директиву *#define DEBUG_MODE*.

Стол работает в режимах Auto, Manual, Nonstop, Video и Rotate 90. В меню Settings пользователь может менять установки.

Сердцем программы является класс Mover, позволяющий осуществлять перемещение в заданную точку (функция Move) или вращать стол с заданной скоростью (функция Run).
### Библиотеки
|Имя|Назначение|Источник|
| ------------ | ------------ | ------------ |
|Encoder|Профессиональная библиотека с использованием ассемблерного кода. Используется для работы с энкодером двигателя. Для оптимизации используется директива *#define ENCODER_OPTIMIZE_INTERRUPTS*|Менеджер библиотек|
|ESP32Encoder|Библиотека для энкодера под ESP32|Менеджер библиотек|
|EncButton|Работа с кнопками и управляющим энкодером|Менеджер библиотек|
|LiquidCrystal_I2C|Работа с дисплеем|Менеджер библиотек|
|PWM|Изменение частоты ШИМ двигателя|https://github.com/terryjmyers/PWM|
### Move
#### Параметры
- *graduations* – количество меток энкодера, на который надо переместить стол. Положительное значение будет перемещать стол по часовой стрелке, отрицательное - против часовой стрелки.
- *maxPWM* – ШИМ-сигнал для перемещения от MIN_PWM до MAX_PWM. Значение по умолчанию равно MAX_PWM.

Функция работает следующим образом: первую половину пути стол разгоняется, а затем тормозит.

![](https://raw.githubusercontent.com/andrey-val-rodin/RotatingTable.Arduino/main/Images/PWM1.png)

Если значение ШИМ превышает MAX_PWM, оно будет обрезано.

![](https://raw.githubusercontent.com/andrey-val-rodin/RotatingTable.Arduino/main/Images/PWM2.png)

Пользователь может задавать значение ускорения (Acceleration) от 1 до 10. Ускорение – это количество меток энкодера от MIN_PWM до MAX_PWM и наоборот (наклонные прямые на графиках). Функция *Settings.getRealAcceleration()* преобразует пользовательские значения от 1 до 10 в реальное количество меток. Опытным путём было установлено, что максимальное ускорение должно занимать не менее 40 меток.

При больших значениях ускорения от 7 до 10 стол часто перескакивает через заданную точку, поэтому тормозить приходится раньше. Это расстояние определяет функция *Mover.getFinalDistance()*. График при этом выглядит следующим образом:

![](https://raw.githubusercontent.com/andrey-val-rodin/RotatingTable.Arduino/main/Images/PWM3.png)

Возвращаемые значения в функции *Mover.getFinalDistance()* были подобраны опытным путём, так что стол совершает минимальное количество ошибок при перемещении. Тем не менее, если ошибка произошла, класс Mover осуществляет так называемую коррекцию, то есть перемещение в заданную точку на минимальном значении ШИМ. Если в этот момент сдвинуть стол назад, стол будет продолжать движение в заданную точку. Если сдвинуть стол так, что он переместится за заданную точку, перемещение завершается.

После завершения основного перемещения, когда стол пришёл в заданную точку, программа некоторое время ждёт полного окончания движения. Это нужно для принятия решения о необходимости коррекции. Если в момент останова стол испытывает сильную вибрацию, программа будет дожидаться прекращения вибрации.

Наконец, последний уровень предотвращения ошибок – это поправка в режимах Auto и Manual. Если стол был сдвинут во время одного из шагов, это будет учтено при осуществлении следующего шага.

В процессе перемещения можно изменять максимальный ШИМ посредством функции *Mover.changePWM()*.
### Run
#### Параметры
- *pwm* – ШИМ-сигнал и направление движения для перемещения. Положительное значение запускает стол по часовой стрелке, отрицательное – против часовой стрелки. Модуль значения определяет максимальный ШИМ-сигнал для движения и должен быть в пределах от MIN_PWM до MAX_PWM.

Запускает движение с заданным значением ШИМ. В процессе движения можно изменять ШИМ посредством функции *Mover.changePWM()*.

### Скорость перемещения в Nonstop режиме
Программа запоминает одно значение для всех значений steps. Это частота срабатывания затвора. Соответственно, для разных значений steps стол будет вращаться с разной скоростью для того, чтобы гарантировать одну и ту же частоту срабатывания затвора. На самых первых и последних значениях steps это не всегда возможно.
Чтобы заставить алгоритмы пересчёта работать правильно на разных типах столов, нужно менять функцию *Settings.getTimeOfTurn()*. Получить нужные значения можно при помощи скетча https://github.com/andrey-val-rodin/RotatingTable.Arduino/tree/main/Measures/pwm2speed.

### Начало движения
В начале движения (Run или Move) программа определяет, начал ли стол движение. Это необходимо, поскольку стол может быть сильно нагружен, а трение покоя больше трения скольжения. Для этого в классе Mover есть таймер, начальное значение которого равно 100 миллисекундам. Если это время превышено, начальное значение ШИМ (_minPWM) начинает инкрементироваться.

Начальный ШИМ не может увеличиваться бесконечно, верний лимит равен 100. Этот лимит может быть уменьшен, если, например, задано перемещение на малое расстояние или ускорение слишком мало.
