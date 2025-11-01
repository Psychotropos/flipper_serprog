#![crate_type = "staticlib"]
#![no_main]
#![no_std]

use core::ffi::*;
use core::mem::*;
use core::time::Duration;

extern crate flipperzero_rt;
extern crate flipperzero_alloc;
extern crate alloc;

use alloc::string::String;
use alloc::vec::Vec;
use flipperzero_sys::*;
use flipperzero_rt::{entry, manifest};

manifest!(name = "Serial Flash Programmer", app_version = 1, has_icon = true, icon = "flipper_serprog.icon");
entry!(main);

struct SerprogData {
    view_port: *mut ViewPort,
    event_queue: *mut FuriMessageQueue,
    worker_thread: *mut FuriThread,
    trx_thread: *mut FuriThread,
    rx_stream: *mut FuriStreamBuffer,
    tx_stream: *mut FuriStreamBuffer,
    prescaler_value: u32,
    running: bool,
}

const RECORD_GUI: *const c_char = "gui\0".as_ptr();
const RECORD_CLI_VCP: *const c_char = "cli_vcp\0".as_ptr();

#[allow(dead_code, non_camel_case_types)]
const S_ACK: u8 = 0x06;
const S_NAK: u8 = 0x15;

const CDC_DATA_SZ: usize = 0x40;

// From flashrom:
const BUS_SPI: u8 = 1 << 3;

const USB_VCP_CHANNEL: u8 = 1;

const FURI_FLAG_WAIT_FOREVER: u32 = 0xFFFFFFFF;
const FURI_FLAG_ERROR: u32 = 0x80000000;
const FURI_FLAG_WAIT_ANY: u32 = 0;

#[repr(u32)]
#[allow(non_camel_case_types)]
enum CR1Bits {
    SPI_CR1_BR = 0x0038,
    SPI_CR1_BR_0 = 0x0008,
    SPI_CR1_BR_1 = 0x0010,
    SPI_CR1_BR_2 = 0x0020,
}

#[repr(u32)]
#[allow(dead_code, non_camel_case_types)]
// External SPI is on APB2, so the base frequency is 64MHz. The comments reflect this.
enum PrescalerValues {
    LL_SPI_BAUDRATEPRESCALER_DIV2 = 0,
    /* 32MHz */
    LL_SPI_BAUDRATEPRESCALER_DIV4 = CR1Bits::SPI_CR1_BR_0 as u32,
    /* 16MHz */
    LL_SPI_BAUDRATEPRESCALER_DIV8 = CR1Bits::SPI_CR1_BR_1 as u32,
    /* 8MHz */
    LL_SPI_BAUDRATEPRESCALER_DIV16 = (CR1Bits::SPI_CR1_BR_1 as u32 | CR1Bits::SPI_CR1_BR_0 as u32),
    /* 4MHz */
    LL_SPI_BAUDRATEPRESCALER_DIV32 = CR1Bits::SPI_CR1_BR_2 as u32,
    /* 2MHz */
    LL_SPI_BAUDRATEPRESCALER_DIV64 = (CR1Bits::SPI_CR1_BR_2 as u32 | CR1Bits::SPI_CR1_BR_0 as u32),
    /* 1MHz */
    LL_SPI_BAUDRATEPRESCALER_DIV128 = (CR1Bits::SPI_CR1_BR_2 as u32 | CR1Bits::SPI_CR1_BR_1 as u32),
    /* 500KHz */
    LL_SPI_BAUDRATEPRESCALER_DIV256 = (CR1Bits::SPI_CR1_BR_2 as u32
        | CR1Bits::SPI_CR1_BR_1 as u32
        | CR1Bits::SPI_CR1_BR_0 as u32),
    /* 250KHz */
}

#[repr(u32)]
#[allow(dead_code, non_camel_case_types)]
enum PrescalerValuesInHz {
    LL_SPI_BAUDRATEPRESCALER_DIV2 = 32000000,
    /* 32MHz */
    LL_SPI_BAUDRATEPRESCALER_DIV4 = 16000000,
    /* 16MHz */
    LL_SPI_BAUDRATEPRESCALER_DIV8 = 8000000,
    /* 8MHz */
    LL_SPI_BAUDRATEPRESCALER_DIV16 = 4000000,
    /* 4MHz */
    LL_SPI_BAUDRATEPRESCALER_DIV32 = 2000000,
    /* 2MHz */
    LL_SPI_BAUDRATEPRESCALER_DIV64 = 1000000,
    /* 1MHz */
    LL_SPI_BAUDRATEPRESCALER_DIV128 = 500000,
    /* 500KHz */
    LL_SPI_BAUDRATEPRESCALER_DIV256 = 250000,
    /* 250KHz */
}

#[repr(u8)]
#[allow(dead_code, non_camel_case_types)]
enum SerprogCommands {
    S_CMD_NOP = 0x00,
    S_CMD_Q_IFACE = 0x01,
    S_CMD_Q_CMDMAP = 0x02,
    S_CMD_Q_PGMNAME = 0x03,
    S_CMD_Q_SERBUF = 0x04,
    S_CMD_Q_BUSTYPE = 0x05,
    S_CMD_Q_CHIPSIZE = 0x06,
    S_CMD_Q_OPBUF = 0x07,
    S_CMD_Q_WRNMAXLEN = 0x08,
    S_CMD_R_BYTE = 0x09,
    S_CMD_R_NBYTES = 0x0A,
    S_CMD_O_INIT = 0x0B,
    S_CMD_O_WRITEB = 0x0C,
    S_CMD_O_WRITEN = 0x0D,
    S_CMD_O_DELAY = 0x0E,
    S_CMD_O_EXEC = 0x0F,
    S_CMD_SYNCNOP = 0x10,
    S_CMD_Q_RDNMAXLEN = 0x11,
    S_CMD_S_BUSTYPE = 0x12,
    S_CMD_O_SPIOP = 0x13,
    S_CMD_S_SPI_FREQ = 0x14,
    S_CMD_S_PIN_STATE = 0x15,
}

const SUPPORTED_COMMANDS: u32 = (1 << SerprogCommands::S_CMD_NOP as u32)
    | (1 << SerprogCommands::S_CMD_Q_IFACE as u32)
    | (1 << SerprogCommands::S_CMD_Q_CMDMAP as u32)
    | (1 << SerprogCommands::S_CMD_Q_PGMNAME as u32)
    | (1 << SerprogCommands::S_CMD_Q_SERBUF as u32)
    | (1 << SerprogCommands::S_CMD_Q_BUSTYPE as u32)
    | (1 << SerprogCommands::S_CMD_SYNCNOP as u32)
    | (1 << SerprogCommands::S_CMD_O_SPIOP as u32)
    | (1 << SerprogCommands::S_CMD_S_BUSTYPE as u32)
    | (1 << SerprogCommands::S_CMD_S_SPI_FREQ as u32);

#[repr(u32)]
enum WorkerEvents {
    CdcRx = (1 << 0),
    CdcStop = (1 << 1),
    CdcTxStream = (1 << 2),
    CdcTx = (1 << 3),
}

const WORKER_ALL_CDC_EVENTS: u32 = WorkerEvents::CdcRx as u32
    | WorkerEvents::CdcStop as u32
    | WorkerEvents::CdcTxStream as u32
    | WorkerEvents::CdcTx as u32;

fn main(args: Option<&CStr>) -> i32 {
    unsafe {
        _entry(args)
    }
}

unsafe fn _entry(_args: Option<&CStr>) -> i32 {
    let mut state = SerprogData {
        view_port: view_port_alloc(),
        event_queue: furi_message_queue_alloc(8, size_of::<InputEvent>() as u32),
        trx_thread: 0 as *mut FuriThread,
        worker_thread: 0 as *mut FuriThread,
        rx_stream: furi_stream_buffer_alloc(5 * CDC_DATA_SZ, 1),
        tx_stream: furi_stream_buffer_alloc(5 * CDC_DATA_SZ, 1),
        prescaler_value: PrescalerValues::LL_SPI_BAUDRATEPRESCALER_DIV32 as u32,
        running: true,
    };

    let mut cdc_cb: CdcCallbacks = CdcCallbacks {
        tx_ep_callback: Some(vcp_on_cdc_tx_complete),
        rx_ep_callback: Some(vcp_on_cdc_rx),
        state_callback: Some(vcp_state_callback),
        ctrl_line_callback: Some(vcp_on_cdc_control_line),
        config_callback: Some(vcp_on_line_config),
    };

    furi_hal_spi_bus_handle_init(&furi_hal_spi_bus_handle_external);
    furi_hal_usb_unlock();

    if USB_VCP_CHANNEL == 0 {
        let cli_vcp = furi_record_open(RECORD_CLI_VCP) as *mut CliVcp;
        cli_vcp_disable(cli_vcp);
        furi_record_close(RECORD_CLI_VCP);
    }

    if !furi_hal_usb_set_config(
        if USB_VCP_CHANNEL == 0 {
            &mut usb_cdc_single as *mut FuriHalUsbInterface
        } else {
            &mut usb_cdc_dual as *mut FuriHalUsbInterface
        },
        0 as *mut c_void,
    ) {
        panic!("Failed to set USB config on init");
    }

    furi_hal_cdc_set_callbacks(
        USB_VCP_CHANNEL,
        &mut cdc_cb as *mut CdcCallbacks,
        &mut state as *mut SerprogData as *mut c_void,
    );

    state.worker_thread = furi_create_thread(
        "SerprogUsbProcThread\0".into(),
        2048,
        &mut state as *mut SerprogData as *mut c_void,
        Some(usb_process_thread_callback),
        true,
    );

    state.trx_thread = furi_create_thread(
        "SerprogUsbTRxThread\0".into(),
        2048,
        &mut state as *mut SerprogData as *mut c_void,
        Some(usb_trx_thread_callback),
        false,
    );

    furi_thread_flags_set(
        furi_thread_get_id(state.trx_thread),
        WorkerEvents::CdcRx as u32,
    );

    furi_thread_start(state.trx_thread);

    view_port_draw_callback_set(
        state.view_port,
        Some(draw_callback),
        state.view_port as *mut c_void,
    );
    view_port_input_callback_set(state.view_port, Some(input_callback), state.event_queue as *mut c_void);

    let gui: *mut Gui = furi_record_open(RECORD_GUI) as *mut Gui;
    gui_add_view_port(gui, state.view_port, GuiLayerFullscreen);

    let mut event: MaybeUninit<InputEvent> = MaybeUninit::uninit();

    while state.running {
        if furi_message_queue_get(
            state.event_queue,
            event.as_mut_ptr() as *mut c_void,
            100,
        ) == FuriStatusOk
        {
            let event = event.assume_init();
            if event.type_ == InputTypePress || event.type_ == InputTypeRepeat
            {
                #[allow(non_upper_case_globals)]
                match event.key {
                    InputKeyBack => {
                        state.running = false;
                        break;
                    }
                    _ => (),
                }
            }
        }
        view_port_update(state.view_port);
    }

    furi_hal_cdc_set_callbacks(USB_VCP_CHANNEL, 0 as *mut CdcCallbacks, 0 as *mut c_void);

    furi_hal_usb_unlock();
    if !furi_hal_usb_set_config(
        &mut usb_cdc_single as *mut FuriHalUsbInterface,
        0 as *mut c_void,
    ) {
        panic!("Failed to reset USB config on destruction");
    }

    if USB_VCP_CHANNEL == 0 {
        let cli = furi_record_open(RECORD_CLI_VCP) as *mut CliVcp;
        cli_vcp_enable(cli);
        furi_record_close(RECORD_CLI_VCP);
    }

    furi_thread_flags_set(
        furi_thread_get_id(state.trx_thread),
        WorkerEvents::CdcStop as u32,
    );
    furi_thread_join(state.trx_thread);
    furi_thread_free(state.trx_thread);

    furi_thread_join(state.worker_thread);
    furi_thread_free(state.worker_thread);

    furi_stream_buffer_free(state.rx_stream);
    furi_stream_buffer_free(state.tx_stream);

    furi_hal_spi_bus_handle_deinit(&furi_hal_spi_bus_handle_external);

    view_port_enabled_set(state.view_port, false);
    gui_remove_view_port(gui, state.view_port);
    view_port_free(state.view_port);
    furi_message_queue_free(state.event_queue);

    furi_record_close(RECORD_GUI);

    0
}

unsafe fn set_spi_baud_rate(handle: &FuriHalSpiBusHandle, prescaler_value: u32) {
    let bus = handle.bus.as_mut().unwrap();
    let spi = bus.spi.as_mut().unwrap();
    spi.CR1 = (spi.CR1 & !(CR1Bits::SPI_CR1_BR as u32)) | prescaler_value;
}

pub unsafe extern "C" fn input_callback(
    input_event: *mut InputEvent,
    event_queue: *mut c_void,
) {
    furi_message_queue_put(
        event_queue as *mut FuriMessageQueue,
        input_event as *mut c_void,
        FURI_FLAG_WAIT_FOREVER,
    );
}

pub unsafe extern "C" fn draw_callback(canvas: *mut Canvas, _context: *mut c_void) {
    canvas_draw_str(canvas, 39, 31, "SPI Flasher\0".as_ptr());
}

unsafe fn usb_process_packet(state: &mut SerprogData) {
    let mut data: [u8; CDC_DATA_SZ] = [0; CDC_DATA_SZ];

    loop {
        let receive_length =
            furi_stream_buffer_receive(state.rx_stream, data.as_mut_ptr() as *mut c_void, 1, 0);
        if receive_length == 0 {
            // Nothing here for us - stop busy looping.
            // We will re-enter this once a CdcRx event is received by the processing thread anyway.
            return;
        }

        let _send_length = 0;

        let command: SerprogCommands = transmute(data[0]);

        let send_length: usize = (|| match command {
            SerprogCommands::S_CMD_NOP => {
                data[0] = S_ACK;
                return 1;
            }
            SerprogCommands::S_CMD_Q_IFACE => {
                data[0] = S_ACK;
                data[1] = 0x01;
                return 3;
            }
            SerprogCommands::S_CMD_Q_CMDMAP => {
                data[0] = S_ACK;
                *(data.as_mut_ptr().add(1) as *mut u32) = SUPPORTED_COMMANDS;
                return 33;
            }
            SerprogCommands::S_CMD_Q_PGMNAME => {
                data[0] = S_ACK;
                data[1] = b'f';
                data[2] = b'u';
                data[3] = b'r';
                data[4] = b'i';
                return 17;
            }
            SerprogCommands::S_CMD_Q_SERBUF => {
                data[0] = S_ACK;
                *(data.as_mut_ptr().add(1) as *mut u16) = 0xFFFF as u16;
                return 3;
            }
            SerprogCommands::S_CMD_Q_BUSTYPE => {
                data[0] = S_ACK;
                data[1] = BUS_SPI;
                return 2;
            }
            SerprogCommands::S_CMD_SYNCNOP => {
                data[0] = S_NAK;
                data[1] = S_ACK;
                return 2;
            }
            SerprogCommands::S_CMD_O_SPIOP => {
                if blocking_furi_stream_buffer_receive(
                    state.rx_stream,
                    data.as_mut_ptr() as *mut c_void,
                    6,
                    10,
                ) != 6
                {
                    data[0] = S_NAK;
                    return 1;
                }

                let write_length: usize = *(data.as_mut_ptr() as *const usize) & 0x00FFFFFF;
                let mut read_length: usize =
                    *(data.as_mut_ptr().add(3) as *const usize) & 0x00FFFFFF;

                let mut read_buffer: Vec<u8> = Vec::with_capacity(read_length);
                read_buffer.set_len(read_length);

                let mut write_buffer: Vec<u8> = Vec::with_capacity(write_length);
                write_buffer.set_len(write_length);

                if blocking_furi_stream_buffer_receive(
                    state.rx_stream,
                    write_buffer.as_mut_ptr() as *mut c_void,
                    write_length,
                    1000,
                ) != write_length
                {
                    data[0] = S_NAK;
                    return 1;
                }

                data[0] = S_ACK;
                signal_usb_cdc_send(&state, data.as_mut_ptr(), 1);

                furi_hal_spi_acquire(&furi_hal_spi_bus_handle_external);

                if state.prescaler_value != PrescalerValues::LL_SPI_BAUDRATEPRESCALER_DIV32 as u32 {
                    set_spi_baud_rate(&furi_hal_spi_bus_handle_external, state.prescaler_value);
                }

                furi_hal_spi_bus_tx(
                    &furi_hal_spi_bus_handle_external,
                    write_buffer.as_mut_ptr(),
                    write_length,
                    5000,
                );
                furi_hal_spi_bus_rx(
                    &furi_hal_spi_bus_handle_external,
                    read_buffer.as_mut_ptr(),
                    read_length,
                    5000,
                );
                furi_hal_spi_release(
                    &furi_hal_spi_bus_handle_external,
                );

                let mut read_index = 0;
                while read_length > 0 {
                    let bytes_to_write = core::cmp::min(CDC_DATA_SZ, read_length);
                    signal_usb_cdc_send(
                        state,
                        read_buffer[read_index..read_index + bytes_to_write].as_mut_ptr(),
                        bytes_to_write,
                    );
                    read_length -= bytes_to_write;
                    read_index += bytes_to_write;
                }

                return 0;
            }
            SerprogCommands::S_CMD_S_BUSTYPE => {
                if blocking_furi_stream_buffer_receive(
                    state.rx_stream,
                    data.as_mut_ptr() as *mut c_void,
                    1,
                    10,
                ) != 1
                {
                    data[0] = S_NAK;
                    return 1;
                }

                data[0] = if (data[0] | BUS_SPI) == BUS_SPI {
                    S_ACK
                } else {
                    S_NAK
                };
                return 1;
            }
            SerprogCommands::S_CMD_S_SPI_FREQ => {
                if blocking_furi_stream_buffer_receive(
                    state.rx_stream,
                    data.as_mut_ptr() as *mut c_void,
                    4,
                    10,
                ) != 4
                {
                    data[0] = S_NAK;
                    return 1;
                }

                let freq = *(data.as_ptr() as *const u32);
                if freq == 0 {
                    data[0] = S_NAK;
                    return 1;
                } else {
                    let mut prescaler_value = PrescalerValues::LL_SPI_BAUDRATEPRESCALER_DIV256;
                    let mut prescaler_value_in_hz =
                        PrescalerValuesInHz::LL_SPI_BAUDRATEPRESCALER_DIV256;

                    if freq >= 2000000 {
                        prescaler_value = PrescalerValues::LL_SPI_BAUDRATEPRESCALER_DIV32;
                        prescaler_value_in_hz = PrescalerValuesInHz::LL_SPI_BAUDRATEPRESCALER_DIV32;
                    } else if freq >= 1000000 {
                        prescaler_value = PrescalerValues::LL_SPI_BAUDRATEPRESCALER_DIV64;
                        prescaler_value_in_hz = PrescalerValuesInHz::LL_SPI_BAUDRATEPRESCALER_DIV64;
                    } else if freq >= 500000 {
                        prescaler_value = PrescalerValues::LL_SPI_BAUDRATEPRESCALER_DIV128;
                        prescaler_value_in_hz =
                            PrescalerValuesInHz::LL_SPI_BAUDRATEPRESCALER_DIV128;
                    } else if freq < 500000 {
                        prescaler_value = PrescalerValues::LL_SPI_BAUDRATEPRESCALER_DIV256;
                        prescaler_value_in_hz =
                            PrescalerValuesInHz::LL_SPI_BAUDRATEPRESCALER_DIV256;
                    }

                    state.prescaler_value = prescaler_value as u32;
                    data[0] = S_ACK;
                    *(data.as_mut_ptr().add(1) as *mut u32) = prescaler_value_in_hz as u32;
                    return 5;
                }
            }
            _ => {
                data[0] = S_NAK;
                return 1;
            }
        })();

        if send_length > 0 {
            signal_usb_cdc_send(&state, data.as_mut_ptr(), send_length);
        }
    }
}

unsafe fn signal_usb_cdc_send(state: &SerprogData, data: *mut u8, length: usize) {
    furi_stream_buffer_send(
        state.tx_stream,
        data as *mut c_void,
        length,
        FURI_FLAG_WAIT_FOREVER,
    );
    furi_thread_flags_set(
        furi_thread_get_id(state.trx_thread),
        WorkerEvents::CdcTxStream as u32,
    );
}

unsafe fn blocking_furi_stream_buffer_receive(
    stream_buffer: *mut FuriStreamBuffer,
    data: *mut c_void,
    length: usize,
    timeout: usize,
) -> usize {
    let mut remaining = length;
    let mut index = 0;

    let timer = furi_hal_cortex_timer_get(Duration::from_millis(timeout as u64).as_micros() as u32);

    while remaining > 0 {
        let timer_clone: FuriHalCortexTimer = transmute_copy(&timer);
        if furi_hal_cortex_timer_is_expired(timer_clone) {
            furi_stream_buffer_reset(stream_buffer);
            return index;
        }

        let count = furi_stream_buffer_receive(
            stream_buffer,
            data.add(index),
            core::cmp::min(CDC_DATA_SZ, remaining),
            0,
        );
        remaining -= count;
        index += count;
    }

    length
}

pub unsafe extern "C" fn usb_process_thread_callback(context: *mut c_void) -> i32 {
    let state = (context as *mut SerprogData).as_mut().unwrap();

    loop {
        let events = furi_thread_flags_wait(
            WORKER_ALL_CDC_EVENTS,
            FURI_FLAG_WAIT_ANY,
            FURI_FLAG_WAIT_FOREVER,
        );
        if (events & FURI_FLAG_ERROR) > 0 {
            panic!("FuriFlagError set in usb_process_thread_callback's furi_thread_flags_wait");
        }

        if (events & (WorkerEvents::CdcStop as u32)) > 0 {
            break;
        }

        if (events & (WorkerEvents::CdcRx as u32)) > 0 {
            usb_process_packet(state);
        }
    }

    0
}

pub unsafe extern "C" fn usb_trx_thread_callback(context: *mut c_void) -> i32 {
    let mut data: [u8; CDC_DATA_SZ] = [0; CDC_DATA_SZ];
    let mut tx_idle = true;
    let mut last_tx_pkt_length = 0;
    let state = (context as *mut SerprogData).as_ref().unwrap();

    loop {
        let mut events = furi_thread_flags_wait(
            WORKER_ALL_CDC_EVENTS,
            FURI_FLAG_WAIT_ANY,
            FURI_FLAG_WAIT_FOREVER,
        );
        if (events & FURI_FLAG_ERROR) > 0 {
            panic!("FuriFlagError set in usb_trx_thread_callback's furi_thread_flags_wait");
        }

        if (events & (WorkerEvents::CdcStop as u32)) > 0 {
            furi_thread_flags_set(
                furi_thread_get_id(state.worker_thread),
                WorkerEvents::CdcStop as u32,
            );
            break;
        }

        if (events & (WorkerEvents::CdcTxStream as u32)) > 0 {
            if tx_idle {
                events |= WorkerEvents::CdcTx as u32;
            }
        }

        if events & (WorkerEvents::CdcTx as u32) > 0 {
            let length = furi_stream_buffer_receive(
                state.tx_stream,
                data.as_mut_ptr() as *mut c_void,
                CDC_DATA_SZ,
                0,
            );
            if length > 0 {
                tx_idle = false;
                furi_hal_cdc_send(USB_VCP_CHANNEL, data.as_mut_ptr(), length as u16);
                last_tx_pkt_length = length;
            } else {
                if last_tx_pkt_length == 64 {
                    furi_hal_cdc_send(USB_VCP_CHANNEL, 0 as *mut u8, 0);
                } else {
                    tx_idle = true;
                }
                last_tx_pkt_length = 0;
            }
        }

        if (events & (WorkerEvents::CdcRx as u32)) > 0 {
            let length =
                furi_hal_cdc_receive(USB_VCP_CHANNEL, data.as_mut_ptr(), CDC_DATA_SZ as u16)
                    as usize;
            if length > 0 {
                furi_stream_buffer_send(
                    state.rx_stream,
                    &data as *const u8 as *const c_void,
                    length,
                    FURI_FLAG_WAIT_FOREVER,
                );

                furi_thread_flags_set(
                    furi_thread_get_id(state.worker_thread),
                    WorkerEvents::CdcRx as u32,
                );
            }
        }
    }

    0
}

pub unsafe extern "C" fn vcp_on_cdc_tx_complete(context: *mut c_void) {
    furi_thread_flags_set(
        furi_thread_get_id((*(context as *mut SerprogData)).trx_thread),
        WorkerEvents::CdcTx as u32,
    );
}

pub unsafe extern "C" fn vcp_on_cdc_rx(context: *mut c_void) {
    furi_thread_flags_set(
        furi_thread_get_id((*(context as *mut SerprogData)).trx_thread),
        WorkerEvents::CdcRx as u32,
    );
}

pub unsafe extern "C" fn vcp_state_callback(_context: *mut c_void, _state: CdcState) {}

pub unsafe extern "C" fn vcp_on_cdc_control_line(_context: *mut c_void, _state: CdcCtrlLine) {}

pub unsafe extern "C" fn vcp_on_line_config(
    _context: *mut c_void,
    _config: *mut usb_cdc_line_coding,
) {}

unsafe fn furi_create_thread(
    thread_name: String,
    stack_size: usize,
    context: *mut c_void,
    callback: FuriThreadCallback,
    start_immediately: bool,
) -> *mut FuriThread {
    let c_thread_name: &CStr = &CStr::from_bytes_with_nul(thread_name.as_bytes()).unwrap();
    let thread = furi_thread_alloc();

    furi_thread_set_name(thread, c_thread_name.as_ptr());
    furi_thread_set_stack_size(thread, stack_size);
    furi_thread_set_context(thread, context);
    furi_thread_set_callback(thread, callback);

    if start_immediately {
        furi_thread_start(thread);
    }

    thread
}
