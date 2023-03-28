/*
 * Copyright (c) 2006-2022, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2018-03-30     chenyong     first version
 * 2018-04-12     chenyong     add client implement
 * 2018-08-17     chenyong     multiple client support
 * 2021-03-17     Meco Man     fix a buf of leaking memory
 * 2021-07-14     Sszl         fix a buf of leaking memory
 * 2022-12-22     lihongquan   port to esp32
 * 2023-3-26      lihongquan   port to zephyr
 */

#include <at.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define LOG_TAG                        "at.client"
#include "at_adapter.h"

#if defined LOG_MODULE_REGISTER
LOG_MODULE_REGISTER(at_client, LOG_LEVEL_INF);
#endif

#define AT_RESP_END_OK                 "OK"
#define AT_RESP_END_ERROR              "ERROR"
#define AT_RESP_END_FAIL               "FAIL"
#define AT_END_CR_LF                   "\r\n"

static struct at_client at_client_table[AT_CLIENT_NUM_MAX];

extern rt_size_t at_utils_send(rt_device_t dev,
                               rt_off_t pos,
                               const void *buffer,
                               rt_size_t size);
extern rt_size_t at_vprintfln(rt_device_t device, const char *format, va_list args);
extern void at_print_raw_cmd(const char *type, const char *cmd, rt_size_t size);
extern const char *at_get_last_cmd(rt_size_t *cmd_size);
static at_resp_status_t client_parse(at_client_t client);

/**
 * Create response object.
 *
 * @param buf_size the maximum response buffer size
 * @param line_num the number of setting response lines
 *         = 0: the response data will auto return when received 'OK' or 'ERROR'
 *        != 0: the response data will return when received setting lines number data
 * @param timeout the maximum response time
 *
 * @return != RT_NULL: response object
 *          = RT_NULL: no memory
 */
at_response_t at_create_resp(rt_size_t buf_size, rt_size_t line_num, rt_int32_t timeout)
{
    at_response_t resp = RT_NULL;

    resp = (at_response_t)rt_calloc(1, sizeof(struct at_response));
    if (resp == RT_NULL)
    {
        LOG_E("AT create response object failed! No memory for response object!");
        return RT_NULL;
    }

    resp->buf = (char *)rt_calloc(1, buf_size);
    if (resp->buf == RT_NULL)
    {
        LOG_E("AT create response object failed! No memory for response buffer!");
        rt_free(resp);
        return RT_NULL;
    }

    resp->buf_size = buf_size;
    resp->line_num = line_num;
    resp->line_counts = 0;
    resp->timeout = timeout;

    return resp;
}

/**
 * Delete and free response object.
 *
 * @param resp response object
 */
void at_delete_resp(at_response_t resp)
{
    if (resp && resp->buf)
    {
        rt_free(resp->buf);
    }

    if (resp)
    {
        rt_free(resp);
        resp = RT_NULL;
    }
}

/**
 * Set response object information
 *
 * @param resp response object
 * @param buf_size the maximum response buffer size
 * @param line_num the number of setting response lines
 *         = 0: the response data will auto return when received 'OK' or 'ERROR'
 *        != 0: the response data will return when received setting lines number data
 * @param timeout the maximum response time
 *
 * @return  != RT_NULL: response object
 *           = RT_NULL: no memory
 */
at_response_t at_resp_set_info(at_response_t resp, rt_size_t buf_size, rt_size_t line_num, rt_int32_t timeout)
{
    char *p_temp;
    RT_ASSERT(resp);

    if (resp->buf_size != buf_size)
    {
        resp->buf_size = buf_size;

        p_temp = (char *)rt_realloc(resp->buf, buf_size);
        if (p_temp == RT_NULL)
        {
            LOG_D("No memory for realloc response buffer size(%d).", buf_size);
            return RT_NULL;
        }
        else
        {
            resp->buf = p_temp;
        }
    }

    resp->line_num = line_num;
    resp->timeout = timeout;

    return resp;
}

/**
 * Get one line AT response buffer by line number.
 *
 * @param resp response object
 * @param resp_line line number, start from '1'
 *
 * @return != RT_NULL: response line buffer
 *          = RT_NULL: input response line error
 */
const char *at_resp_get_line(at_response_t resp, rt_size_t resp_line)
{
    char *resp_buf = resp->buf;
    char *resp_line_buf = RT_NULL;
    rt_size_t line_num = 1;

    RT_ASSERT(resp);

    if (resp_line > resp->line_counts || resp_line <= 0)
    {
        LOG_E("AT response get line failed! Input response line(%d) error!", resp_line);
        return RT_NULL;
    }

    for (line_num = 1; line_num <= resp->line_counts; line_num++)
    {
        if (resp_line == line_num)
        {
            resp_line_buf = resp_buf;

            return resp_line_buf;
        }

        resp_buf += strlen(resp_buf) + 1;
    }

    return RT_NULL;
}

/**
 * Get one line AT response buffer by keyword
 *
 * @param resp response object
 * @param keyword query keyword
 *
 * @return != RT_NULL: response line buffer
 *          = RT_NULL: no matching data
 */
const char *at_resp_get_line_by_kw(at_response_t resp, const char *keyword)
{
    char *resp_buf = resp->buf;
    char *resp_line_buf = RT_NULL;
    rt_size_t line_num = 1;

    RT_ASSERT(resp);
    RT_ASSERT(keyword);

    for (line_num = 1; line_num <= resp->line_counts; line_num++)
    {
        if (strstr(resp_buf, keyword))
        {
            resp_line_buf = resp_buf;

            return resp_line_buf;
        }

        resp_buf += strlen(resp_buf) + 1;
    }

    return RT_NULL;
}

/**
 * Get and parse AT response buffer arguments by line number.
 *
 * @param resp response object
 * @param resp_line line number, start from '1'
 * @param resp_expr response buffer expression
 *
 * @return -1 : input response line number error or get line buffer error
 *          0 : parsed without match
 *         >0 : the number of arguments successfully parsed
 */
int at_resp_parse_line_args(at_response_t resp, rt_size_t resp_line, const char *resp_expr, ...)
{
    va_list args;
    int resp_args_num = 0;
    const char *resp_line_buf = RT_NULL;

    RT_ASSERT(resp);
    RT_ASSERT(resp_expr);

    if ((resp_line_buf = at_resp_get_line(resp, resp_line)) == RT_NULL)
    {
        return -1;
    }

    va_start(args, resp_expr);

    resp_args_num = vsscanf(resp_line_buf, resp_expr, args);

    va_end(args);

    return resp_args_num;
}

/**
 * Get and parse AT response buffer arguments by keyword.
 *
 * @param resp response object
 * @param keyword query keyword
 * @param resp_expr response buffer expression
 *
 * @return -1 : input keyword error or get line buffer error
 *          0 : parsed without match
 *         >0 : the number of arguments successfully parsed
 */
int at_resp_parse_line_args_by_kw(at_response_t resp, const char *keyword, const char *resp_expr, ...)
{
    va_list args;
    int resp_args_num = 0;
    const char *resp_line_buf = RT_NULL;

    RT_ASSERT(resp);
    RT_ASSERT(resp_expr);

    if ((resp_line_buf = at_resp_get_line_by_kw(resp, keyword)) == RT_NULL)
    {
        return -1;
    }

    va_start(args, resp_expr);

    resp_args_num = vsscanf(resp_line_buf, resp_expr, args);

    va_end(args);

    return resp_args_num;
}

/**
 * Send commands to AT server and wait response.
 *
 * @param client current AT client object
 * @param resp AT response object, using RT_NULL when you don't care response
 * @param cmd_expr AT commands expression
 *
 * @return 0 : success
 *        -1 : response status error
 *        -2 : wait timeout
 *        -7 : enter AT CLI mode
 * result = at_exec_cmd(resp, "AT+CIFSR");
 */
int at_obj_exec_cmd(at_client_t client, at_response_t resp, const char *cmd_expr, ...)
{
    va_list args;
    rt_size_t cmd_size = 0;
    rt_err_t result = RT_EOK;
    const char *cmd = RT_NULL;

    RT_ASSERT(cmd_expr);

    if (client == RT_NULL)
    {
        LOG_E("input AT Client object is NULL, please create or get AT Client object!");
        return -RT_ERROR;
    }

    /* check AT CLI mode */
    if (client->status == AT_STATUS_CLI && resp)
    {
        return -RT_EBUSY;
    }

    rt_mutex_take(client->lock, RT_WAITING_FOREVER);

    client->resp_status = AT_RESP_OK;

    if (resp != RT_NULL)
    {
        resp->buf_len = 0;
        resp->line_counts = 0;
    }

    client->resp = resp;
    com_flush(client->device);

    va_start(args, cmd_expr);
    at_vprintfln(client->device, cmd_expr, args);
    va_end(args);

    if (resp != RT_NULL)
    {
        client_parse(client);

        if (AT_RESP_TIMEOUT == client->resp_status)
        {
            cmd = at_get_last_cmd(&cmd_size);
            LOG_W("execute command (%.*s) timeout (%d ticks)!", cmd_size, cmd, resp->timeout);
            result = -RT_ETIMEOUT;
            goto __exit;
        }
        if (client->resp_status != AT_RESP_OK)
        {
            cmd = at_get_last_cmd(&cmd_size);
            LOG_E("execute command (%.*s) failed!", cmd_size, cmd);
            result = -RT_ERROR;
            goto __exit;
        }
    }

__exit:
    client->resp = RT_NULL;

    rt_mutex_release(client->lock);

    return result;
}

/**
 * Send data to AT server, send data don't have end sign(eg: \r\n).
 *
 * @param client current AT client object
 * @param buf   send data buffer
 * @param size  send fixed data size
 *
 * @return >0: send data size
 *         =0: send failed
 */
rt_size_t at_client_obj_send(at_client_t client, const char *buf, rt_size_t size)
{
    rt_size_t len;

    RT_ASSERT(buf);

    if (client == RT_NULL)
    {
        LOG_E("input AT Client object is NULL, please create or get AT Client object!");
        return 0;
    }

#ifdef AT_PRINT_RAW_CMD
    at_print_raw_cmd("sendline", buf, size);
#endif

    rt_mutex_take(client->lock, RT_WAITING_FOREVER);

    len = at_utils_send(client->device, 0, buf, size);

    rt_mutex_release(client->lock);

    return len;
}

static int at_client_getchar(at_client_t client, char *ch, uint32_t timeout)
{
    /* getchar */
    if (com_read(client->device, ch, 1, timeout) <= 0)
    {
        return -RT_ETIMEOUT;
    }

    return RT_EOK;
}

/**
 *  AT client set end sign.
 *
 * @param client current AT client object
 * @param ch the end sign, can not be used when it is '\0'
 */
void at_obj_set_end_sign(at_client_t client, char ch)
{
    if (client == RT_NULL)
    {
        LOG_E("input AT Client object is RT_NULL, please create or get AT Client object!");
        return;
    }

    client->end_sign = ch;
}

/**
 * get AT client object by AT device name.
 *
 * @dev_name AT client device name
 *
 * @return AT client object
 */
at_client_t at_client_get(const char *dev_name)
{
    int idx = 0;

    RT_ASSERT(dev_name);

    for (idx = 0; idx < AT_CLIENT_NUM_MAX; idx++)
    {
        if (rt_strcmp(com_device_name(at_client_table[idx].device), dev_name) == 0)
        {
            return &at_client_table[idx];
        }
    }

    return RT_NULL;
}

/**
 * get first AT client object in the table.
 *
 * @return AT client object
 */
at_client_t at_client_get_first(void)
{
    if (at_client_table[0].device == RT_NULL)
    {
        return RT_NULL;
    }

    return &at_client_table[0];
}

static int at_recv_readline(at_client_t client, uint32_t timeout)
{
    rt_size_t read_len = 0;
    char ch = 0, last_ch = 0;
    rt_bool_t is_full = RT_FALSE;
    uint32_t end_time = rt_tick_get() + timeout;
    uint32_t remaining_time = timeout;

    rt_memset(client->recv_line_buf, 0x00, client->recv_bufsz);
    client->recv_line_len = 0;

    while (1)
    {
        /* getchar */
        at_client_getchar(client, &ch, remaining_time);

        if (read_len < client->recv_bufsz)
        {
            client->recv_line_buf[read_len++] = ch;
            client->recv_line_len = read_len;
        }
        else
        {
            is_full = RT_TRUE;
        }

        /* is newline or URC data */
        if ((ch == '\n' && last_ch == '\r') || (client->end_sign != 0 && ch == client->end_sign))
        {
            if (is_full)
            {
                LOG_E("read line failed. The line data length is out of buffer size(%d)!", client->recv_bufsz);
                rt_memset(client->recv_line_buf, 0x00, client->recv_bufsz);
                client->recv_line_len = 0;
                return -RT_EFULL;
            }
            break;
        }
        last_ch = ch;

        if (RT_WAITING_FOREVER != timeout)
        {
            remaining_time = end_time - rt_tick_get();
        }

        if ((remaining_time > timeout) || (0 == remaining_time))
        {
            break;
        }
    }

#ifdef AT_PRINT_RAW_CMD
    at_print_raw_cmd("recvline", client->recv_line_buf, read_len);
#endif

    return read_len;
}

static at_resp_status_t client_parse(at_client_t client)
{
    uint32_t end_time = rt_tick_get() + client->resp->timeout;
    uint32_t remaining_time = client->resp->timeout;

    while (1)
    {
        if (at_recv_readline(client, remaining_time) > 0)
        {
            if (client->resp != RT_NULL)
            {
                at_response_t resp = client->resp;

                char end_ch = client->recv_line_buf[client->recv_line_len - 1];

                /* current receive is response */
                client->recv_line_buf[client->recv_line_len - 1] = '\0';
                if (resp->buf_len + client->recv_line_len < resp->buf_size)
                {
                    /* copy response lines, separated by '\0' */
                    rt_memcpy(resp->buf + resp->buf_len, client->recv_line_buf, client->recv_line_len);

                    /* update the current response information */
                    resp->buf_len += client->recv_line_len;
                    resp->line_counts++;
                }
                else
                {
                    client->resp_status = AT_RESP_BUFF_FULL;
                    LOG_E("Read response buffer failed. The Response buffer size is out of buffer size(%d)!", resp->buf_size);
                }

                /* check response result */
                if ((client->end_sign != 0) && (end_ch == client->end_sign) && (resp->line_num == 0))
                {
                    /* get the end sign, return response state END_OK.*/
                    client->resp_status = AT_RESP_OK;
                }
                else if (rt_memcmp(client->recv_line_buf, AT_RESP_END_OK, rt_strlen(AT_RESP_END_OK)) == 0 && resp->line_num == 0)
                {
                    /* get the end data by response result, return response state END_OK. */
                    client->resp_status = AT_RESP_OK;
                }
                else if (rt_strstr(client->recv_line_buf, AT_RESP_END_ERROR) || (rt_memcmp(client->recv_line_buf, AT_RESP_END_FAIL, rt_strlen(AT_RESP_END_FAIL)) == 0))
                {
                    client->resp_status = AT_RESP_ERROR;
                }
                else if (resp->line_counts == resp->line_num && resp->line_num)
                {
                    /* get the end data by response line, return response state END_OK.*/
                    client->resp_status = AT_RESP_OK;
                }
                else
                {
                    continue;
                }

                client->resp = RT_NULL;
                break;
            }
            else
            {
                //                LOG_D("unrecognized line: %.*s", client->recv_line_len, client->recv_line_buf);
            }
        }

        if (RT_WAITING_FOREVER != client->resp->timeout)
        {
            remaining_time = end_time - rt_tick_get();
        }

        if ((remaining_time > client->resp->timeout) || (0 == remaining_time))
        {
            client->resp_status = AT_RESP_TIMEOUT;
            break;
        }
    }

    return client->resp_status;
}

/* I don't need this function */
#if 0
static rt_err_t at_client_rx_ind(rt_device_t dev, rt_size_t size)
{
    int idx = 0;

    for (idx = 0; idx < AT_CLIENT_NUM_MAX; idx++)
    {
        if (at_client_table[idx].device == dev && size > 0)
        {
            rt_sem_release(at_client_table[idx].rx_notice);
        }
    }

    return RT_EOK;
}
#endif

/* initialize the client object parameters */
static int at_client_para_init(at_client_t client)
{
#define AT_CLIENT_LOCK_NAME "at_c"
#define AT_CLIENT_SEM_NAME "at_cs"
#define AT_CLIENT_RESP_NAME "at_cr"
#define AT_CLIENT_THREAD_NAME "at_clnt"

    int result = RT_EOK;
    static int at_client_num = 0;
    char name[RT_NAME_MAX];

    client->status = AT_STATUS_UNINITIALIZED;

    client->recv_line_len = 0;
    client->recv_line_buf = (char *)rt_calloc(1, client->recv_bufsz);
    if (client->recv_line_buf == RT_NULL)
    {
        LOG_E("AT client initialize failed! No memory for receive buffer.");
        result = -RT_ENOMEM;
        goto __exit;
    }

    rt_snprintf(name, RT_NAME_MAX, "%s%d", AT_CLIENT_LOCK_NAME, at_client_num);
    client->lock = rt_mutex_create(name, RT_IPC_FLAG_PRIO);
    if (client->lock == RT_NULL)
    {
        LOG_E("AT client initialize failed! at_client_recv_lock create failed!");
        result = -RT_ENOMEM;
        goto __exit;
    }

    /* I don't need the notice from interrupt */
#if 0
    rt_snprintf(name, RT_NAME_MAX, "%s%d", AT_CLIENT_SEM_NAME, at_client_num);
    client->rx_notice = rt_sem_create(name, 0, RT_IPC_FLAG_FIFO);
    if (client->rx_notice == RT_NULL)
    {
        LOG_E("AT client initialize failed! at_client_notice semaphore create failed!");
        result = -RT_ENOMEM;
        goto __exit;
    }
#endif

__exit:
    if (result != RT_EOK)
    {
        if (client->lock)
        {
            rt_mutex_delete(client->lock);
        }
#if 0
        if (client->rx_notice)
        {
            rt_sem_delete(client->rx_notice);
        }
#endif

        if (client->device)
        {
#if 0
            rt_device_close(client->device);
#endif
            client->device = NULL;
        }

        if (client->recv_line_buf)
        {
            rt_free(client->recv_line_buf);
        }

        rt_memset(client, 0x00, sizeof(struct at_client));
    }
    else
    {
        at_client_num++;
    }

    return result;
}

/**
 * AT client initialize.
 *
 * @param dev AT client device name
 * @param recv_bufsz the maximum number of receive buffer length
 *
 * @return 0 : initialize success
 *        -1 : initialize failed
 *        -5 : no memory
 */
int at_client_init(rt_device_t dev, rt_size_t recv_bufsz)
{
    int idx = 0;
    int result = RT_EOK;
    at_client_t client = RT_NULL;

    RT_ASSERT(dev);
    RT_ASSERT(recv_bufsz > 0);

    if (at_client_get(com_device_name(dev)) != RT_NULL)
    {
        return result;
    }

    for (idx = 0; idx < AT_CLIENT_NUM_MAX && at_client_table[idx].device; idx++)
        ;

    if (idx >= AT_CLIENT_NUM_MAX)
    {
        LOG_E("AT client initialize failed! Check the maximum number(%d) of AT client.", AT_CLIENT_NUM_MAX);
        result = -RT_EFULL;
        goto __exit;
    }

    client = &at_client_table[idx];
    client->recv_bufsz = recv_bufsz;

    client->device = dev;

    result = at_client_para_init(client);
    if (result != RT_EOK)
    {
        goto __exit;
    }

__exit:
    if (result == RT_EOK)
    {
        client->status = AT_STATUS_INITIALIZED;

        LOG_I("AT client(V%s) on device %s initialize success.", AT_SW_VERSION, com_device_name(dev));
    }
    else
    {
        LOG_E("AT client(V%s) on device %s initialize failed(%d).", AT_SW_VERSION, com_device_name(dev), result);
    }

    return result;
}
