/*
 * ESPRESSIF MIT License
 *
 * Copyright (c) 2018 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on all ESPRESSIF SYSTEMS products, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */


#ifndef _IOT_BUTTON_H_
#define _IOT_BUTTON_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <driver/gpio.h>
#include <freertos/portmacro.h>
typedef void (* button_cb)(void *);
typedef void *button_handle_t;

typedef enum {
    BUTTON_ACTIVE_HIGH = 1,    //кнопка активный уровень: высокий
    BUTTON_ACTIVE_LOW = 0,     //кнопка активный уровень: низкий
} button_active_t;

typedef enum {
    BUTTON_CB_PUSH = 0,   //событие обратного вызова нажатия кнопки
    BUTTON_CB_RELEASE,    //событие обратного вызова отпускания кнопки
    BUTTON_CB_TAP,        //событие обратного вызова кнопки(не будет срабатывать, если уже есть событие "PRESS")
    BUTTON_CB_SERIAL,     //событие обратного вызова последовательного триггера кнопки
} button_cb_type_t;

/**
 * @brief инициализация функций кнопок
 *
 * @param gpio_num Индекс GPIO pin-кода, используемого кнопкой
 * @param active_level кнопка аппаратного активного уровня.
 * Для "BUTTON_ACTIVE_LOW"это означает, что при нажатии кнопки GPIO 
 *будет считывать низкий уровень.
 * @return Дескриптор button_handle_t для созданного объекта button 
 * или NULL в случае ошибки.
 */
button_handle_t iot_button_create(gpio_num_t gpio_num, button_active_t active_level);

/**
 * @brief Register a callback function for a serial trigger event.
 *
 * @param btn_handle handle of the button object
 * @param start_after_sec define the time after which to start serial trigger action
 * @param interval_tick serial trigger interval
 * @param cb callback function for "TAP" action.
 * @param arg Parameter for callback function
 * @note
 *        Button callback functions execute in the context of the timer service task.
 *        It is therefore essential that button callback functions never attempt to block.
 *        For example, a button callback function must not call vTaskDelay(), vTaskDelayUntil(),
 *        or specify a non zero block time when accessing a queue or a semaphore.
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t iot_button_set_serial_cb(button_handle_t btn_handle, uint32_t start_after_sec, TickType_t interval_tick, button_cb cb, void *arg);

/**
 * @brief Register a callback function for a button_cb_type_t action.
 *
 * @param btn_handle handle of the button object
 * @param type callback function type
 * @param cb callback function for "TAP" action.
 * @param arg Parameter for callback function
 * @note
 *        Button callback functions execute in the context of the timer service task.
 *        It is therefore essential that button callback functions never attempt to block.
 *        For example, a button callback function must not call vTaskDelay(), vTaskDelayUntil(),
 *        or specify a non zero block time when accessing a queue or a semaphore.
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t iot_button_set_evt_cb(button_handle_t btn_handle, button_cb_type_t type, button_cb cb, void *arg);

/**
 * @brief Callbacks invoked as timer events occur while button is pressed.
 *        Example: If a button is configured for 2 sec, 5 sec and 7 sec callbacks and if the button is pressed for 6 sec then 2 sec callback would be invoked at 2 sec event and 5 sec callback would be invoked at 5 sec event
 *
 * @param btn_handle handle of the button object
 * @param press_sec the callback function would be called if you press the button for a specified period of time
 * @param cb callback function for "PRESS and HOLD" action.
 * @param arg Parameter for callback function
 *
 * @note
 *        Button callback functions execute in the context of the timer service task.
 *        It is therefore essential that button callback functions never attempt to block.
 *        For example, a button callback function must not call vTaskDelay(), vTaskDelayUntil(),
 *        or specify a non zero block time when accessing a queue or a semaphore.
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t iot_button_add_on_press_cb(button_handle_t btn_handle, uint32_t press_sec, button_cb cb, void *arg);

/**
 * @brief Single callback invoked according to the latest timer event on button release.
 *        Example: If a button is configured for 2 sec, 5 sec and 7 sec callbacks and if the button is released at 6 sec then only 5 sec callback would be invoked
 *
 * @param btn_handle handle of the button object
 * @param press_sec the callback function would be called if you press the button for a specified period of time
 * @param cb callback function for "PRESS and RELEASE" action.
 * @param arg Parameter for callback function
 *
 * @note
 *        Button callback functions execute in the context of the timer service task.
 *        It is therefore essential that button callback functions never attempt to block.
 *        For example, a button callback function must not call vTaskDelay(), vTaskDelayUntil(),
 *        or specify a non zero block time when accessing a queue or a semaphore.
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t iot_button_add_on_release_cb(button_handle_t btn_handle, uint32_t press_sec, button_cb cb, void *arg);

/**
 * @brief Delete button object and free memory
 * @param btn_handle handle of the button object
 *
 * @return
 *     - ESP_OK Success
 *     - ESP_FAIL Parameter error
 */
esp_err_t iot_button_delete(button_handle_t btn_handle);

/**
 * @brief Remove callback
 *
 * @param btn_handle The handle of the button object
 * @param type callback function event type
 *
 * @return
 *     - ESP_OK Success
 */
esp_err_t iot_button_rm_cb(button_handle_t btn_handle, button_cb_type_t type);

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

/**
 * class of button
 * simple usage:
 * CButton* btn = new CButton(BUTTON_IO_NUM, BUTTON_ACTIVE_LEVEL, BUTTON_SERIAL_TRIGGER, 3);
 * btn->add_cb(BUTTON_CB_PUSH, button_tap_cb, (void*) push, 50 / portTICK_PERIOD_MS);
 * btn->add_custom_cb(5, button_press_5s_cb, NULL);
 * ......
 * delete btn;
 */
class CButton
{
private:
    button_handle_t m_btn_handle;

    /**
     * prevent copy constructing
     */
    CButton(const CButton &);
    CButton &operator = (const CButton &);
public:

    /**
     * @brief constructor of CButton
     *
     * @param gpio_num GPIO index of the pin that the button uses
     * @param active_level button hardware active level.
     *        For "BUTTON_ACTIVE_LOW" it means when the button pressed, the GPIO will read low level.
     */
    CButton(gpio_num_t gpio_num, button_active_t active_level = BUTTON_ACTIVE_LOW);

    ~CButton();

    /**
     * @brief Register a callback function for a button_cb_type_t action.
     *
     * @param type callback function type
     * @param cb callback function for "TAP" action.
     * @param arg Parameter for callback function
     * @note
     *        Button callback functions execute in the context of the timer service task.
     *        It is therefore essential that button callback functions never attempt to block.
     *        For example, a button callback function must not call vTaskDelay(), vTaskDelayUntil(),
     *        or specify a non zero block time when accessing a queue or a semaphore.
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Parameter error
     */
    esp_err_t set_evt_cb(button_cb_type_t type, button_cb cb, void *arg);

    /**
     * @brief Register a callback function for a serial trigger event.
     *
     * @param btn_handle handle of the button object
     * @param start_after_sec define the time after which to start serial trigger action
     * @param interval_tick serial trigger interval
     * @param cb callback function for "TAP" action.
     * @param arg Parameter for callback function
     * @note
     *        Button callback functions execute in the context of the timer service task.
     *        It is therefore essential that button callback functions never attempt to block.
     *        For example, a button callback function must not call vTaskDelay(), vTaskDelayUntil(),
     *        or specify a non zero block time when accessing a queue or a semaphore.
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Parameter error
     */
    esp_err_t set_serial_cb(button_cb cb, void *arg, TickType_t interval_tick, uint32_t start_after_sec);

    /**
     * @brief Callbacks invoked as timer events occur while button is pressed
     *
     * @param press_sec the callback function would be called if you press the button for a specified period of time
     * @param cb callback function for "PRESS and HOLD" action.
     * @param arg Parameter for callback function
     *
     * @note
     *        Button callback functions execute in the context of the timer service task.
     *        It is therefore essential that button callback functions never attempt to block.
     *        For example, a button callback function must not call vTaskDelay(), vTaskDelayUntil(),
     *        or specify a non zero block time when accessing a queue or a semaphore.
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Parameter error
     */
    esp_err_t add_on_press_cb(uint32_t press_sec, button_cb cb, void *arg);

    /**
     * @brief Single callback invoked according to the latest timer event on button release.
     *
     * @param press_sec the callback function would be called if you press the button for a specified period of time
     * @param cb callback function for "PRESS and RELEASE" action.
     * @param arg Parameter for callback function
     *
     * @note
     *        Button callback functions execute in the context of the timer service task.
     *        It is therefore essential that button callback functions never attempt to block.
     *        For example, a button callback function must not call vTaskDelay(), vTaskDelayUntil(),
     *        or specify a non zero block time when accessing a queue or a semaphore.
     * @return
     *     - ESP_OK Success
     *     - ESP_FAIL Parameter error
     */
    esp_err_t add_on_release_cb(uint32_t press_sec, button_cb cb, void *arg);

    /**
     * @brief Remove callback
     *
     * @param type callback function event type
     *
     * @return
     *     - ESP_OK Success
     */
    esp_err_t rm_cb(button_cb_type_t type);
};
#endif

#endif
