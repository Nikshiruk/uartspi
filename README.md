Шлюз

Аппаратная платформа: STM32
Программная платформа: FreeRTOS

Устройство имеет USART и SPI в режиме master. Необходимо реализовать двунаправленный шлюз USART ↔️ SPI. 

- Данные передаются сообщениями, протокол можно определить произвольно
- Сообщения на USART и SPI могут приходить асинхронно (у SPI slave устройства может возникнуть необходимость передать сообщение в любой момент)
- Работа с USART и SPI должна быть реализована в разных задачах

1) размер фрэйма динамический (значение длины 16 бит)
2) формат фрейма - строка (оканчиваются NULL), в SPI slave устройстве нет никаких внутренних регистров и прочих адресаций. Начиная с первого же байта оно начинает слать строку. Если устройству нечего передавать, то начиная с первого же байта оно шлет NULL. И также начиная с первого же байта оно принимает строку.
Со стороны уарта также приходят просто строки с символом \0 на  конце.
3) размер очереди - можете установить по 10 в обе стороны.
4) скорости - на ваше усмотрение

Результат - в виде Github репозитория

Описание решения.
Задача предусматривает FREERTOS с разными потоками для UART и SPI, поэтому создаются два потока (помимо основного) - UART и SPI, которые считывают данные для отправки каждый из своей очереди. 
Основное отличие интерфейсов в том, что для чтения данных по SPI необходимо что-то отправлять. Для записи и чтения SPI организованы два буфера небольшого размера (по 16 байт). Для накопления
данных в очереди используется простейший формат строки, первые два байта (16 бит по условию задачи) отведены для размера данных в фрейме. Фреймы следуют друг за другом без разделителей.
Сами фреймы в очередь не помещаются, в соответствующую очередь записывается указатель на начало фрейма - 16-разрядную длину его данных. Сами данные хранятся в двух кольцевых буферах.
Суть алгоритма проста - если данные приходят по шине UART - они помещаются в буфер для передачи по SPI, а указатель на эти данные помещается в очередь. И наоборот, если данные пришли по SPI,
они помещаются в буфер UART, а указатель в очередь для передачи по UART. Два потока пеередают свои данные, если очередь не пуста. Анализ принятых данныз SPI делается по завершению приема и
передачи буфера, а анализ данных UART при неизвестной длине фреймов делается по приему каждого байта. Помимо ожидания появления очередного фрейма на отправку в очереди каждый поток ожидает
также завершение отправки предыдущего фрейма. Помещение данных в очередь реализовано в CallBack - функциях: 1 окончание приема данных по UART (один байт) - зациклена на прием очередного байта;
2 окончание приема/передачи SPI (перебираются 16 байт) - зациклена на прием очередных 16 байт, если очередь на отправку SPI пуста, отправляются нули для чтения. Определение готовности шины UART
для передачи очередного фрейма (за один раз с помощью DMA передаются все данные фрейма) делается с помощью флага готовности к передаче, взводимого в CallBack - функции по завершению передачи UART
Программа выполнена с помощью среды CubeMX, все описанные алгоритмы находятся в файле main.c (в папке Core\Src), все остальные файлы могут быть сгенерированы CubeMX под различные системы
разработки на основе проекта uartspi.ioc для микроконтроллера STM32H750.
