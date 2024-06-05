# stm32-distance-ivanyshyn_ahitoliev

Team: Andrii Ahitoliev, Ihor Ivanyshyn, Anastasiia Pelekh

**Configuration**

Sensor connections:


PD9 налаштований на Output і названий TRIG. PD10 на EXTI10(interrupt) і названий ECHOI.


В NVIC налаштуваннях, сетимо line interrupts для ECHOI і сетимо переривання на Falling/Rising edge trigger detection.


LCD connections:


SCK pin підключений на PA5, RST - PC4, DIN - PA7, CS(CE) - PC5, DC - PB0. GND і VCC під'єднані на землю і живлення через піни на платі.


З додаткових елементів - додані переривання і вивід за нявності збоїв або завеликої для сенсора відстані.
