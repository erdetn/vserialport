// Copyright(C) 2021 Erdet Nasufi. All rights reserved.

module vsp

#include <stddef.h>

#flag -I .
#flag -I /usr/include

pub enum Return { // C.sp_return
	ok = C.SP_OK
	invalid_arguments = C.SP_ERR_ARG
	fail 		  = C.SP_ERR_FAIL
	memory_error      = C.SP_ERR_MEM
	not_supported     = C.SP_ERR_SUPP
}

pub enum Mode { // C.sp_mode
	read       = C.SP_MODE_READ
	write      = C.SP_MODE_WRITE
	read_write = C.SP_MODE_READ_WRITE
}

pub enum Event { // C.sp_event
	received_ready = C.SP_EVENT_RX_READY
	transmit_ready = C.SP_EVENT_TX_READY
	event_error    = C.SP_EVENT_ERROR
}

pub enum Buffer { // C.sp_buffer
	input  = C.SP_BUF_INPUT
	output = C.SP_BUF_OUTPUT
	both   = C.SP_BUF_BOTH
}

pub enum Parity { // C.sp_parity
	invalid = C.SP_PARITY_INVALID
	@none   = C.SP_PARITY_NONE
	odd     = C.SP_PARITY_ODD
	event   = C.SP_PARITY_EVEN
	mark    = C.SP_PARITY_MARK
	space   = C.SP_PARITY_SPACE
}

pub enum Rts { // C.sp_rts
	invalid      = C.SP_RTS_INVALID
	off          = C.SP_RTS_OFF
	on           = C.SP_RTS_ON
	flow_control = C.SP_RTS_FLOW_CONTROL
};

pub enum Cts { // C.sp_cts
	invalide     = C.SP_CTS_INVALID
	ignore       = C.SP_CTS_IGNORE
	flow_control = C.SP_CTS_FLOW_CONTROL
};

pub enum Dtr { // C.sp_dtr
	invalide     = C.SP_DTR_INVALID
	off          = C.SP_DTR_OFF
	on           = C.SP_DTR_ON
	flow_control = C.SP_DTR_FLOW_CONTROL
};

pub enum Dsr { // C.sp_dsr
	invalide     = C.SP_DSR_INVALID
	ignore       = C.SP_DSR_IGNORE
	flow_control = C.SP_DSR_FLOW_CONTROL
};

pub enum XonXoff { // C.sp_xonxoff
	invalide    = C.SP_XONXOFF_INVALID
	disabled    = C.SP_XONXOFF_DISABLED
	@in	    = C.SP_XONXOFF_IN
	out 	    = C.SP_XONXOFF_OUT
	inout 	    = C.SP_XONXOFF_INOUT
}

pub enum FlowControl { // C.sp_flowcontrol
	@none     = C.SP_FLOWCONTROL_NONE
	xon_xoff  = C.SP_FLOWCONTROL_XONXOFF
	rts_cts   = C.SP_FLOWCONTROL_RTSCTS
	dtr_dsr   = C.SP_FLOWCONTROL_DTRDSR
}

pub enum Signal { // C.sp_signal
	cts   = C.SP_SIG_CTS
	dsr   = C.SP_SIG_DSR
	dcd   = C.SP_SIG_DCD
	ri    = C.SP_SIG_RI
}

pub enum Transport { // C.sp_transport
	native    = C.SP_TRANSPORT_NATIVE
	usb       = C.SP_TRANSPORT_USB
	bluetooth = C.SP_TRANSPORT_BLUETOOTH
}

struct C.sp_port{}
pub struct Port {
	ptr &C.sp_port
}

struct C.sp_port_config{}
pub struct PortConfiguration{
	ptr &C.sp_port_config
}

struct C.sp_event_set {
	void *handles;
	handles voidptr
	masks   &C.Event
	count   u32
}

fn C.sp_get_port_by_name(portname &char, port_ptr &&C.sp_port) C.sp_return

fn C.sp_free_port(&C.sp_port)

fn C.sp_list_ports(struct sp_port ***list_ptr);

fn C.sp_open(&C.sp_port, flags Mode) C.sp_return

fn C.sp_close(port &C.sp_port) C.sp_return

fn C.sp_get_port_name(port &C.sp_port) &char

fn C.sp_get_port_description(port &C.sp_port) &char

fn C.sp_get_port_transport(port &C.sp_port) C.sp_transport

fn C.sp_get_port_usb_bus_address(port &C.sp_port, usb_bus &int, usb_address &int) C.sp_return

fn C.sp_get_port_usb_vid_pid(port &C.sp_port, usb_vid &int, usb_pid &int) C.sp_return

fn C.sp_get_port_usb_manufacturer(port &C.sp_port) &char

fn C.sp_get_port_usb_product(port &C.sp_port) &char

fn C.sp_get_port_usb_serial(port &C.sp_port) &char

fn C.sp_get_port_bluetooth_address(port &C.sp_port) &char

fn C.sp_get_port_handle(port &C.sp_port, result_ptr voidptr) C.sp_return

fn C.sp_new_config(&&C.sp_port_config) C.sp_return

fn C.sp_free_config(&C.sp_port_config)

fn C.sp_get_config(port &C.sp_port, config &C.sp_port_config) C.sp_return

fn C.sp_set_config(port &C.sp_port, config &C.sp_port_config) C.sp_return

fn C.sp_set_baudrate(port &C.sp_port, baudrate int) C.sp_return

fn C.sp_get_config_baudrate(config &C.sp_port_config, baudrate_ptr &int) C.sp_return

fn C.sp_set_config_baudrate(config &C.sp_port_config, baudrate int) C.sp_return

fn C.sp_set_bits(port &C.sp_port, bits int) C.sp_return

fn C.sp_get_config_bits(config &C.sp_port_config, bits_ptr &int) C.sp_return

fn C.sp_set_config_bits(config &C.sp_port_config, bits int) C.sp_return

fn C.sp_set_parity(port &C.sp_port, parity C.sp_parity) C.sp_return

fn C.sp_get_config_parity(config &C.sp_port_config, parity_ptr &C.sp_parity) C.sp_return

fn C.sp_set_config_parity(config &C.sp_port_config, parity C.sp_parity) C.sp_return

fn C.sp_set_stopbits(port &C.sp_port, stopbits int) C.sp_return

fn C.sp_get_config_stopbits(config &C.sp_port_config, stopbits_ptr &int) C.sp_return

fn C.sp_set_config_stopbits(config &C.sp_port_config, stopbits int) C.sp_return

fn C.sp_set_rts(port &C.sp_port, enum sp_rts rts) C.sp_return

fn C.sp_get_config_rts(config &C.sp_port_config, rts_ptr &C.sp_rts) C.sp_return

fn C.sp_set_config_rts(config &C.sp_port_config, rts C.sp_rts) C.sp_return

fn C.sp_set_cts(port &C.sp_port, cts C.sp_cts) C.sp_return

fn C.sp_get_config_cts(config &C.sp_port_config, cts_ptr &C.sp_cts) C.sp_return

fn C.sp_set_config_cts(config &C.sp_port_config, cts C.sp_cts) C.sp_return

fn C.sp_set_dtr(port &C.sp_port, dtr C.sp_dtr) C.sp_return

fn C.sp_get_config_dtr(config &C.sp_port_config, dtr_ptr &C.sp_dtr) C.sp_return

fn C.sp_set_config_dtr(config &C.sp_port_config, dtr C.sp_dtr) C.sp_return

fn C.sp_set_dsr(port &C.sp_port, dsr C.sp_dsr) C.sp_return

fn C.sp_get_config_dsr(config &C.sp_port_config, dsr_ptr &C.sp_dsr) C.sp_return

fn C.sp_set_config_dsr(config &C.sp_port_config, dsr C.sp_dsr) C.sp_return

fn C.sp_set_xon_xoff(port &C.sp_port, xon_xoff C.sp_xonxoff) C.sp_return

fn C.sp_get_config_xon_xoff(config &C.sp_port_config, xon_xoff_ptr &C.sp_xonxoff) C.sp_return

fn C.sp_set_config_xon_xoff(config &C.sp_port_config, xon_xoff &C.sp_xonxoff) C.sp_return

fn C.sp_set_config_flowcontrol(config &C.sp_port_config, flowcontrol C.sp_flowcontrol) C.sp_return

fn C.p_set_flowcontrol(port &C.sp_port, flowcontrol C.sp_flowcontrol) C.sp_return

fn C.sp_blocking_read(port &C.sp_port, buf voidptr, count C.size_t, timeout_ms u32) C.sp_return

fn C.sp_blocking_read_next(port &C.sp_port, buf voidptr, count C.size_t, timeout_ms u32) C.sp_return

fn C.sp_nonblocking_read(port &C.sp_port, buf voidptr, count C.size_t) C.sp_return

fn C.sp_blocking_write(port &C.sp_port, buf voidptr, count C.size_t, timeout_ms u32) C.sp_return

fn C.sp_nonblocking_write(port &C.sp_port, buf voidptr, count C.size_t) C.sp_return

fn C.sp_input_waiting(port &C.sp_port) C.sp_return

fn C.sp_output_waiting(port &C.sp_port) C.sp_return

fn C.sp_flush(port &C.sp_port, buffers C.sp_buffer) C.sp_return

fn C.sp_drain(port &C.sp_port) C.sp_return

fn C.sp_new_event_set(result_ptr &&C.sp_event_set) C.sp_return

fn C.sp_add_port_events(event_set &C.sp_event_set, port &C.sp_port, mask C.sp_event) C.sp_retur

fn C.sp_wait(event_set &C.sp_event_set, timeout_ms u32) C.sp_return

fn C.sp_free_event_set(event_set &C.sp_event_set)

fn C.sp_get_signals(port &C.sp_port, signal_mask &C.sp_signal) C.sp_return

fn C.sp_start_break(port &C.sp_port) C.sp_return

fn C.sp_end_break(port &C.sp_port) C.sp_return

fn C.sp_last_error_code() int

fn C.sp_last_error_message() &chare

fn C.sp_free_error_message(message &char)

// fn C.sp_set_debug_handler(void (*handler)(char *format, ...));

fn C.sp_get_major_package_version() int

fn C.sp_get_minor_package_version() int

fn C.sp_get_micro_package_version() int

fn C.sp_get_package_version_string() &char

fn C.sp_get_current_lib_version() int

fn C.sp_get_revision_lib_version() int

fn C.sp_get_age_lib_version() int

fn C.sp_get_lib_version_string() &char
