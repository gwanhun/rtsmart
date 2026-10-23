/*
 * AIC8800DC Bluetooth over a K230 UART (H:4).
 *
 * The SDIO transport has no Bluetooth channel. On boards that wire the
 * AIC8800DC Bluetooth HCI UART to a K230 UART (01Studio CanMV K230: uart3 on
 * IO50/IO51), aic8800_firmware.c uploads the BT patch over SDIO and selects
 * the UART port; this file then exposes that UART as a standard
 * /dev/hciX device (rt_bt_hci) so NimBLE / MicroPython `bluetooth` can use it.
 *
 * It also registers /dev/aicbt, a read-only text log of the bring-up stages.
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <rtthread.h>
#include <rtdevice.h>
#include <drivers/bt_hci.h>

#include "aic8800_wifi.h"

#define DBG_TAG "aic8800.btuart"
#define DBG_LVL AIC8800_DBG_LVL
#include <rtdbg.h>

#define BTUART_RX_RING_SIZE   4096U
#define BTUART_PKT_MAX        1100U
#define BTUART_READ_CHUNK     256U

void aic8800_bt_status(const char *format, ...);
rt_size_t aic8800_bt_status_read(rt_off_t position, void *buffer,
                                 rt_size_t size);

struct aic_btuart
{
    struct rt_bt_hci_device hci;
    rt_uint8_t hci_rx_buffer[BTUART_RX_RING_SIZE];
    rt_device_t uart;
    struct rt_semaphore rx_sem;
    rt_thread_t thread;
    volatile rt_bool_t opened;
    /* H:4 parser */
    rt_uint8_t state;      /* 0 type, 1 header, 2 payload */
    rt_uint8_t type;
    rt_uint16_t need;
    rt_uint16_t have;
    rt_uint8_t packet[BTUART_PKT_MAX];
    rt_uint32_t rx_bytes;
    rt_uint32_t rx_packets;
    rt_uint32_t tx_packets;
};

static struct aic_btuart g_btuart;
static struct rt_device g_aicbt_status_device;

static rt_uint16_t btuart_header_length(rt_uint8_t type)
{
    switch (type)
    {
    case RT_BT_HCI_H4_EVT:
        return 2U;
    case RT_BT_HCI_H4_SCO:
        return 3U;
    case RT_BT_HCI_H4_ACL:
    case RT_BT_HCI_H4_ISO:
        return 4U;
    default:
        return 0U;
    }
}

static rt_uint16_t btuart_payload_length(rt_uint8_t type,
                                         const rt_uint8_t *header)
{
    switch (type)
    {
    case RT_BT_HCI_H4_EVT:
        return header[1];
    case RT_BT_HCI_H4_SCO:
        return header[2];
    case RT_BT_HCI_H4_ACL:
        return (rt_uint16_t)(header[2] | (header[3] << 8));
    case RT_BT_HCI_H4_ISO:
        return (rt_uint16_t)((header[2] | (header[3] << 8)) & 0x3fffU);
    default:
        return 0U;
    }
}

static void btuart_deliver(struct aic_btuart *bt)
{
    if (rt_bt_hci_receive(&bt->hci, bt->type, bt->packet, bt->have) == RT_EOK)
    {
        if (bt->rx_packets++ == 0U)
        {
            aic8800_bt_status("uart: first HCI packet type=%u len=%u",
                              bt->type, bt->have);
        }
    }
    bt->state = 0;
}

static void btuart_feed(struct aic_btuart *bt, const rt_uint8_t *data,
                        rt_size_t length)
{
    rt_size_t index;

    for (index = 0; index < length; index++)
    {
        rt_uint8_t byte = data[index];

        if (bt->state == 0)
        {
            rt_uint16_t header = btuart_header_length(byte);

            if (!header)
            {
                continue; /* resynchronise on the next packet indicator */
            }
            bt->type = byte;
            bt->need = header;
            bt->have = 0;
            bt->state = 1;
            continue;
        }
        bt->packet[bt->have++] = byte;
        if (bt->have < bt->need)
        {
            continue;
        }
        if (bt->state == 1)
        {
            rt_uint16_t payload = btuart_payload_length(bt->type, bt->packet);

            if ((rt_size_t)bt->need + payload > BTUART_PKT_MAX)
            {
                bt->state = 0;
                continue;
            }
            bt->need = (rt_uint16_t)(bt->need + payload);
            bt->state = 2;
            if (payload == 0U)
            {
                btuart_deliver(bt);
            }
            continue;
        }
        btuart_deliver(bt);
    }
}

static rt_err_t btuart_rx_indicate(rt_device_t device, rt_size_t size)
{
    (void)device;
    (void)size;
    rt_sem_release(&g_btuart.rx_sem);
    return RT_EOK;
}

static void btuart_rx_thread(void *parameter)
{
    struct aic_btuart *bt = (struct aic_btuart *)parameter;
    rt_uint8_t chunk[BTUART_READ_CHUNK];

    while (1)
    {
        rt_sem_take(&bt->rx_sem, rt_tick_from_millisecond(100));
        while (bt->opened && bt->uart)
        {
            rt_size_t got = rt_device_read(bt->uart, -1, chunk, sizeof(chunk));

            if (!got)
            {
                break;
            }
            bt->rx_bytes += got;
            btuart_feed(bt, chunk, got);
        }
    }
}

static rt_err_t btuart_open(struct rt_bt_hci_device *hci)
{
    struct aic_btuart *bt = &g_btuart;
    struct serial_configure config = RT_SERIAL_CONFIG_DEFAULT;
    rt_err_t result;

    (void)hci;
    if (bt->opened)
    {
        return RT_EOK;
    }
    bt->uart = rt_device_find(AIC8800_DC_BT_UART_NAME);
    if (!bt->uart)
    {
        aic8800_bt_status("uart: %s not found", AIC8800_DC_BT_UART_NAME);
        return -RT_ENOSYS;
    }
    config.baud_rate = AIC8800_DC_BT_UART_BAUD;
    config.bufsz = 4096;
    rt_device_control(bt->uart, RT_DEVICE_CTRL_CONFIG, &config);
    result = rt_device_open(bt->uart,
                            RT_DEVICE_OFLAG_RDWR | RT_DEVICE_FLAG_INT_RX);
    if (result != RT_EOK)
    {
        aic8800_bt_status("uart: open %s failed %d",
                          AIC8800_DC_BT_UART_NAME, result);
        bt->uart = RT_NULL;
        return result;
    }
    /* Re-apply after open: some serial drivers reset the line on open. */
    rt_device_control(bt->uart, RT_DEVICE_CTRL_CONFIG, &config);
    rt_device_set_rx_indicate(bt->uart, btuart_rx_indicate);
    bt->state = 0;
    bt->opened = RT_TRUE;
    if (!bt->thread)
    {
        bt->thread = rt_thread_create("aicbtrx", btuart_rx_thread, bt,
                                      AIC8800_DC_BT_UART_STACK_SIZE,
                                      AIC8800_DC_BT_UART_PRIORITY, 10);
        if (!bt->thread)
        {
            aic8800_bt_status("uart: rx thread create failed");
            return -RT_ENOMEM;
        }
        rt_thread_startup(bt->thread);
    }
    aic8800_bt_status("uart: %s open @%u", AIC8800_DC_BT_UART_NAME,
                      (unsigned int)AIC8800_DC_BT_UART_BAUD);
    return RT_EOK;
}

static rt_err_t btuart_close(struct rt_bt_hci_device *hci)
{
    struct aic_btuart *bt = &g_btuart;

    (void)hci;
    if (!bt->opened)
    {
        return RT_EOK;
    }
    bt->opened = RT_FALSE;
    rt_device_set_rx_indicate(bt->uart, RT_NULL);
    rt_device_close(bt->uart);
    bt->uart = RT_NULL;
    aic8800_bt_status("uart: closed rx_bytes=%u rx_pkts=%u tx_pkts=%u",
                      bt->rx_bytes, bt->rx_packets, bt->tx_packets);
    return RT_EOK;
}

static rt_err_t btuart_write_all(rt_device_t uart, const rt_uint8_t *data,
                                 rt_size_t length)
{
    rt_size_t done = 0;
    rt_uint32_t spins = 0;

    while (done < length)
    {
        rt_size_t wrote = rt_device_write(uart, 0, data + done, length - done);

        if (!wrote)
        {
            if (++spins > 200U)
            {
                return -RT_ETIMEOUT;
            }
            rt_thread_mdelay(1);
            continue;
        }
        done += wrote;
    }
    return RT_EOK;
}

static rt_err_t btuart_send(struct rt_bt_hci_device *hci,
                            rt_uint8_t packet_type, const rt_uint8_t *data,
                            rt_size_t length)
{
    struct aic_btuart *bt = &g_btuart;
    rt_err_t result;

    (void)hci;
    if (!bt->opened || !bt->uart)
    {
        return -RT_EIO;
    }
    result = btuart_write_all(bt->uart, &packet_type, 1);
    if (result == RT_EOK)
    {
        result = btuart_write_all(bt->uart, data, length);
    }
    if (result == RT_EOK && bt->tx_packets++ == 0U)
    {
        aic8800_bt_status("uart: first HCI command sent type=%u len=%u",
                          packet_type, (unsigned int)length);
    }
    return result;
}

static const struct rt_bt_hci_ops g_btuart_ops = {
    .open = btuart_open,
    .close = btuart_close,
    .send = btuart_send,
};

static rt_size_t aicbt_status_read(rt_device_t device, rt_off_t position,
                                   void *buffer, rt_size_t size)
{
    (void)device;
    return aic8800_bt_status_read(position, buffer, size);
}

#ifdef RT_USING_DEVICE_OPS
static const struct rt_device_ops g_aicbt_status_ops = {
    RT_NULL, RT_NULL, RT_NULL, aicbt_status_read, RT_NULL, RT_NULL,
};
#endif

static int aic8800_btuart_init(void)
{
    struct aic_btuart *bt = &g_btuart;
    rt_err_t result;

    rt_memset(bt, 0, sizeof(*bt));
    rt_sem_init(&bt->rx_sem, "aicbtrx", 0, RT_IPC_FLAG_PRIO);

    rt_memset(&g_aicbt_status_device, 0, sizeof(g_aicbt_status_device));
    g_aicbt_status_device.type = RT_Device_Class_Char;
#ifdef RT_USING_DEVICE_OPS
    g_aicbt_status_device.ops = &g_aicbt_status_ops;
#else
    g_aicbt_status_device.read = aicbt_status_read;
#endif
    rt_device_register(&g_aicbt_status_device, "aicbt", RT_DEVICE_FLAG_RDONLY);

    result = rt_bt_hci_register_auto(&bt->hci, &g_btuart_ops,
                                     bt->hci_rx_buffer,
                                     sizeof(bt->hci_rx_buffer), bt);
    if (result != RT_EOK)
    {
        aic8800_bt_status("uart: hci register failed %d", result);
        return result;
    }
    aic8800_bt_status("uart: registered /dev/%s on %s",
                      bt->hci.parent.parent.name, AIC8800_DC_BT_UART_NAME);
    LOG_I("Bluetooth HCI on %s registered as /dev/%s",
          AIC8800_DC_BT_UART_NAME, bt->hci.parent.parent.name);
    return RT_EOK;
}
INIT_COMPONENT_EXPORT(aic8800_btuart_init);
