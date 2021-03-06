/**
  ******************************************************************************
  * @file    LwIP/LwIP_HTTP_Server_Netconn_RTOS/Src/httpser-netconn.c 
  * @author  MCD Application Team
  * @version V1.0.0
  * @date    18-November-2015
  * @brief   Basic http server implementation using LwIP netconn API  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2015 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "lwip/opt.h"
#include "lwip/arch.h"
#include "lwip/api.h"
#include "string.h"
#include "httpserver-netconn.h"
#include "cmsis_os.h"
#include "../webpages/index.h"
#include "temp.h"
#include <queue.h>
#include <math.h>
#include <stdlib.h>


/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define WEBSERVER_THREAD_PRIO    ( tskIDLE_PRIORITY + 4 )

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
u32_t nPageHits = 0;
#define LD1_Pin GPIO_PIN_0
#define LD1_GPIO_Port GPIOB

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

static char float_buf[15];
static xQueueHandle m_queue_handle;

static int block_on_queue(int block_ms) 
{
    const portTickType xTicksToWait = block_ms/portTICK_RATE_MS;
    int data = 0;
    portBASE_TYPE xStatus = xQueueReceive(m_queue_handle, &data, xTicksToWait);
    
    if(xStatus == pdPASS) 
    {
        return data;
    }
    else 
    {
        return -1;
    }
    
}

/**
 * HTTP request debug routine
 */
static int request_print_counter = 0;
static int print_body(char * request, u16_t len) {
    if (request_print_counter >= len) {
        request_print_counter = 0;
    }
    return request[request_print_counter++];
}
/**
 * find the index of the body after consecutive \n
 * @param request
 * @param len
 * @return 
 */
static int get_body_index(char * request, u16_t len)
{
    u16_t body_idx = 0;
    u16_t idx = 0;
    
    for(; idx < len; idx++) {
        if((request[idx] == '\n')) {
            body_idx = idx;
            body_idx++;
        }
    }
    return body_idx;
}


static long int update_leds(char * delimited_led_state) 
{
    long int led_state = strtol(delimited_led_state, NULL, 10);
    
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD1_Pin, led_state & 1);
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD2_Pin, led_state & 2);
    HAL_GPIO_WritePin(LD1_GPIO_Port, LD3_Pin, led_state & 4);

    return led_state;
}

static void float_to_str(float val, char * buf)
{
    int d1 = val; // Get the integer part (678).
    float f2 = val - d1; // Get fractional part (0.01234567).
    int d2 = trunc(f2 * 10000); // Turn into integer (123).
//    float f3 = f2 * 10000 - d2; // Get next fractional part (0.4567).
//    int d3 = trunc(f3 * 10000); // Turn into integer (4567).

    sprintf(buf, "%d.%04d", d1, d2);
}

/**
  * @brief serve tcp connection  
  * @param conn: pointer on connection structure 
  * @retval None
  */
void http_server_serve(struct netconn *conn) 
{
  struct netbuf *inbuf;
  err_t recv_err;
  char* buf;
  u16_t buflen;

  /* Read the data from the port, blocking if nothing yet there. 
   We assume the request (the part we care about) is in one netbuf */
  recv_err = netconn_recv(conn, &inbuf);
  
  if (recv_err == ERR_OK)
  {
    if (netconn_err(conn) == ERR_OK) 
    {
      netbuf_data(inbuf, (void**)&buf, &buflen);
    
      /* Is this an HTTP GET command? (only check the first 5 chars, since
      there are other formats for GET, and we're keeping it very simple )*/
      if ((buflen >=5))
      {
    	  if (strncmp((char const *)buf,"GET /index.html",15)==0) {
    		  netconn_write(conn, (const unsigned char*)index_html, index_html_len, NETCONN_NOCOPY);
    	  }
    	  if (strncmp((char const *)buf,"GET /led1", 9) == 0) {
    		  HAL_GPIO_TogglePin(LD1_GPIO_Port, LD1_Pin);
    	  }
    	  if (strncmp((char const *)buf,"GET /led2", 9) == 0) {
    		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    	  }
    	  if (strncmp((char const *)buf,"GET /led3", 9) == 0) {
    		  HAL_GPIO_TogglePin(LD3_GPIO_Port, LD3_Pin);
    	  }
          if (strncmp((char const *)buf,"POST /test", 10) == 0) 
          {
              //static int buf_incrementor = 10;
    		  
              netconn_write(conn, (const unsigned char *)buf, buflen, NETCONN_NOCOPY);
    	  }
          if(strncmp((char const *)buf, "POST /printreq", 15) == 0) {
              int body_char = print_body(buf, buflen);
              sprintf(buf, "%d %d               \n", request_print_counter, body_char);
              netconn_write(conn, (const unsigned char*)buf, strlen(buf), NETCONN_NOCOPY);
          }
          
    	  if (strncmp((char const *)buf, "GET /irq", 8) == 0)
          {
                int body_offset = 9;//get_body_index(buf, buflen);
                int led_status = update_leds(&buf[body_offset]);
                int switch_count = block_on_queue(100);
                float_to_str(getMCUTemperature(), float_buf);
                sprintf(buf, "%s %d %d                 \n", float_buf, switch_count, led_status);
                netconn_write(conn, (const unsigned char *)buf, strlen(buf), NETCONN_NOCOPY);
    	  }
    	  if (strncmp((char const *)buf,"GET /btn1", 9) == 0) {
              netconn_write(conn, (const unsigned char*)"OFF", 3, NETCONN_NOCOPY);
    	  }
    	  if (strncmp((char const *)buf,"GET /adc", 8) == 0) {
              float_to_str(getMCUTemperature(), float_buf);
              sprintf(buf, "%s °C", float_buf);
              netconn_write(conn, (const unsigned char*)buf, strlen(buf), NETCONN_NOCOPY);
    	  }
      }
    }
  }
  /* Close the connection (server closes in HTTP) */
  netconn_close(conn);
  
  /* Delete the buffer (netconn_recv gives us ownership,
   so we have to make sure to deallocate the buffer) */
  netbuf_delete(inbuf);
}


/**
  * @brief  http server thread 
  * @retval None
  */
static void http_server_netconn_thread()
{ 
  struct netconn *conn, *newconn;
  err_t err, accept_err;
  
  /* Create a new TCP connection handle */
  conn = netconn_new(NETCONN_TCP);
  
  if (conn!= NULL)
  {
    /* Bind to port 80 (HTTP) with default IP address */
    err = netconn_bind(conn, NULL, 80);
    
    if (err == ERR_OK)
    {
      /* Put the connection into LISTEN state */
      netconn_listen(conn);
  
      while(1) 
      {
        /* accept any icoming connection */
        accept_err = netconn_accept(conn, &newconn);
        if(accept_err == ERR_OK)
        {
          /* serve connection */
          http_server_serve(newconn);

          /* delete connection */
          netconn_delete(newconn);
        }
      }
    }
  }
}

/**
  * @brief  Initialize the HTTP server (start its thread) 
  * @param  none
  * @retval None
  */
void http_server_netconn_init()
{
  m_queue_handle = xQueueCreate(100/*len*/,4/*size*/);
  sys_thread_new("HTTP", http_server_netconn_thread, NULL, DEFAULT_THREAD_STACKSIZE, WEBSERVER_THREAD_PRIO);
}

/**
 * 
 * @return the queue handle for button pressed
 */
xQueueHandle http_get_btn_queue() 
{
    return m_queue_handle;
}

