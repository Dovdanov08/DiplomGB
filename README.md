Файлы с исходным кодом по диплому курса от GB "Инженер умных устройств". 
МК (STM32 и ESP32) связаны по UART.
STM32 получает данные с датчиков и команды от ESP32 и отображаем на дисплее.
ESP32 получает данные от STM32 и отправлет на MQTT брокеру и так получает от него команды и передает из STM32.