/*******************************************************************************
 Copyright 2020-2024, Cypress Semiconductor Corporation (an Infineon company) or
 an affiliate of Cypress Semiconductor Corporation.  All rights reserved.

 This software, including source code, documentation and related
 materials ("Software") is owned by Cypress Semiconductor Corporation
 or one of its affiliates ("Cypress") and is protected by and subject to
 worldwide patent protection (United States and foreign),
 United States copyright laws and international treaty provisions.
 Therefore, you may use this Software only as provided in the license
 agreement accompanying the software package from which you
 obtained this Software ("EULA").
 If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
 non-transferable license to copy, modify, and compile the Software
 source code solely for use in connection with Cypress's
 integrated circuit products.  Any reproduction, modification, translation,
 compilation, or representation of this Software except as specified
 above is prohibited without the express written permission of Cypress.

 Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
 EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
 WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
 reserves the right to make changes to the Software without notice. Cypress
 does not assume any liability arising out of the application or use of the
 Software or any product or circuit described in the Software. Cypress does
 not authorize its products for use in any products where a malfunction or
 failure of the Cypress product may reasonably be expected to result in
 significant property damage, injury or death ("High Risk Product"). By
 including Cypress's product in a High Risk Product, the manufacturer
 of such system or application assumes all risk of such use and in doing
 so agrees to indemnify Cypress against all liability.
 *******************************************************************************/

/*******************************************************************************
 *        Header Files
 *******************************************************************************/
#include "cyhal.h"
#include "sensor_task.h"
#include "cy_retarget_io.h"
#include "timers.h"

#include "app_bt_gatt_handler.h"
#include "cycfg_gatt_db.h"

/*******************************************************************************
 *        Macro Definitions
 *******************************************************************************/
#define LTR11_TDET                      (CYBSP_D12)
#define LTR11_PDET                      (CYBSP_D11)

#define POLL_TIMER_IN_MSEC              (500u)
#define POLL_TIMER_FREQ                 (10000)

/* Check if notification is enabled for a valid connection ID */
#define IS_NOTIFIABLE(conn_id, cccd)    (((conn_id)!= 0)? (cccd) & GATT_CLIENT_CONFIG_NOTIFICATION: 0)

#define TARGET_DETECT_INDEX             (0)
#define PHASE_DETECT_INDEX              (1)

#define TARGET_DETECTED                 (0)
#define TARGET_NOT_DETECTED             (1)

#define TARGET_APPROACHING              (1)
#define TARGET_DEPARTING                (0)

/*******************************************************************************
 *        Variable Definitions
 *******************************************************************************/
/* Handle to timer object */
TimerHandle_t timer_handle;

/******************************************************************************
 *                          Function Prototypes
 ******************************************************************************/
void timer_callback(TimerHandle_t xTimer);

/******************************************************************************
 *                          Function Definitions
 ******************************************************************************/
int32_t sensor_task_init()
{
    int32_t result;

    /* Create timer */
    timer_handle = xTimerCreate("timer",
                                pdMS_TO_TICKS(POLL_TIMER_IN_MSEC),
                                pdTRUE,
                                NULL,
                                timer_callback);
    if (NULL == timer_handle)
    {
        return -1;
    }

    /* Start Timer */
    xTimerStart(timer_handle, 0);

    /* Initialize sensor driver here */
    result = cyhal_gpio_init(LTR11_TDET, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, 0);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Error initializing LTR11 TDET pin\n");
        return -2;
    }

    result = cyhal_gpio_init(LTR11_PDET, CYHAL_GPIO_DIR_INPUT, CYHAL_GPIO_DRIVE_NONE, 0);
    if (result != CY_RSLT_SUCCESS)
    {
        printf("Error initializing LTR11 PDET pin\n");
        return -3;
    }

    return 0;
}

void sensor_task(void *pvParameters)
{
    /* TDET = Target Detected
     * if TDET = 0 (low), Target detected
     * if TDET = 1 (high), Target not detected
     * */

    /* PDET = Phase Detection
     * if PDET = 0 (low), Target departing
     * if PDET = 1 (high), Target approaching
     * */

    for (;;)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        /* Read TDET and PDET GPIOs */
        bool TDET_status = cyhal_gpio_read(LTR11_TDET);
        bool PDET_status = cyhal_gpio_read(LTR11_PDET);

        /* Set buffer with target detect and phase detect status */
        app_xensiv_sensor_shield_bgt60ltr11[TARGET_DETECT_INDEX] = TDET_status;
        app_xensiv_sensor_shield_bgt60ltr11[PHASE_DETECT_INDEX] = PDET_status;

        if (TARGET_DETECTED == TDET_status)
        {
            printf("Presence: Detected\n");

            if (TARGET_APPROACHING == PDET_status)
            {
                printf("Direction: Approaching\n\n");
            }
            else
            {
                printf("Direction: Departing\n\n");
            }
        }
        else
        {
            printf("Presence: Not Detected\n\n");
        }

        if (IS_NOTIFIABLE (app_bt_conn_id, app_xensiv_sensor_shield_bgt60ltr11_client_char_config[0]) == 0)
        {
            if(!app_bt_conn_id)
            {
                printf("This device is not connected to a central device\n");
            }
            else
            {
                printf("This device is connected to a central device but\n"
                        "GATT client notifications are not enabled\n");
            }
        }
        else
        {
            wiced_bt_gatt_status_t gatt_status;

            /*
            * Sending notification, set the pv_app_context to NULL, since the
            * data 'app_xensiv_sensor_shield_bgt60ltr11' is not to be freed
            */
            gatt_status = wiced_bt_gatt_server_send_notification(app_bt_conn_id,
                                                                 HDLC_XENSIV_SENSOR_SHIELD_BGT60LTR11_VALUE,
                                                                 app_xensiv_sensor_shield_bgt60ltr11_len,
                                                                 (uint8_t *) app_xensiv_sensor_shield_bgt60ltr11,
                                                                 NULL);

            printf("Sent notification status 0x%x\n", gatt_status);
        }
    }
}

void timer_callback(TimerHandle_t xTimer)
{
    (void) xTimer;

    BaseType_t xHigherPriorityTaskWoken;

    xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(sensor_task_handle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
