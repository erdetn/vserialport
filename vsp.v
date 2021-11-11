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
mut:
	port_name  string
	ptr 	  &C.sp_port
}

fn C.sp_get_port_by_name(portname &char, port_ptr &&C.sp_port) C.sp_return
pub fn new_port(port_name string) Port {
	mut this := Port{}
	this.port_name = port_name
	cfn := &char(port_name.str)
	unsafe {
		C.sp_get_port_by_name(cfn, &this.ptr)
	}
	return this
}

fn C.sp_free_port(&C.sp_port)
pub fn (mut this Port)free() {
	C.sp_free_port(this.ptr)
}

// fn C.sp_list_ports(struct sp_port ***list_ptr);

fn C.sp_open(&C.sp_port, flags C.sp_mode) C.sp_return
pub fn (this Port)open(flags Mode) bool {
	rc := C.sp_open(this.ptr, C.sp_mode(flags))
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_close(port &C.sp_port) C.sp_return
pub fn (this Port)close() bool {
	rc := C.sp_close(this.ptr)
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_get_port_name(port &C.sp_port) &char
pub fn (this Port)name() string {
	cpn := unsafe {
		cstring_to_vstring(C.sp_get_port_name(this.ptr))
	}
	return cpn
}

fn C.sp_get_port_description(port &C.sp_port) &char
pub fn (this Port)description() string {
	cpd := unsafe {
		cstring_to_vstring(C.sp_get_port_description(this.ptr))
	}
}

fn C.sp_get_port_transport(port &C.sp_port) C.sp_transport
pub fn (this Port)transport() Transport {
	pt := Transport(C.sp_get_port_transport(this.ptr))
	return pt
}

pub struct UsbBus {
	usb_bus     int
	usb_address int
}

fn C.sp_get_port_usb_bus_address(port &C.sp_port, usb_bus &int, usb_address &int) C.sp_return
pub fn (this Port)usb_bus() UsbBus {
	ub := int(0)
	ua := int(0)
	rc := unsafe {
		C.sp_get_port_usb_bus_address(this.ptr, &ub, &ua)
	}

	if rc != C.sp_return(C.SP_OK) {
		return UsbBus{0, 0}
	}

	return UsbBus{
		usb_bus:     ub,
		usb_address: ua
	}
}

pub struct UsbID {
	vendor_id  int
	product_id int
}

fn C.sp_get_port_usb_vid_pid(port &C.sp_port, usb_vid &int, usb_pid &int) C.sp_return
pub fn (this Port)usb_id() UsbID {
	vid := int(0)
	pid := int(0)
	rc := unsafe {
		C.sp_get_port_usb_vid_pid(this.ptr, &vid, &pid)
	}
	
	if rc != C.sp_return(C.SP_OK) {
		return UsbID{0, 0}
	}
	return UsbID{
		vendor_id:  vid
		product_id: pid
	}
}

fn C.sp_get_port_usb_manufacturer(port &C.sp_port) &char
pub fn (this Port)usb_manufacturer() string {
	cstr := unsafe {
		cstring_to_vstring(C.sp_get_port_usb_manufacturer(this.ptr))
	}
	return cstr
}

fn C.sp_get_port_usb_product(port &C.sp_port) &char
pub fn (this Port)usb_product() string {
	cstr := unsafe {
		cstring_to_vstring(C.sp_get_port_usb_product(this.ptr))
	}
	return cstr
}

fn C.sp_get_port_usb_serial(port &C.sp_port) &char
pub fn (this Port)usb_serial_number() string {
	cstr := unsafe {
		cstring_to_vstring(C.sp_get_port_usb_serial(this.ptr))
	}
	return cstr
}

fn C.sp_get_port_bluetooth_address(port &C.sp_port) &char
pub fn (this Port)bluetooth_address() string {
	cstr := unsafe {
		cstring_to_vstring(C.sp_get_port_bluetooth_address(this.ptr))
	}
	return cstr
}

fn C.sp_get_port_handle(port &C.sp_port, result_ptr voidptr) C.sp_return
$if windows {
	pub fn (this Port)handler() ?voidptr {
		h := voidptr(0)
		rc := unsafe {
			C.sp_get_port_handle(this.ptr, &h)
		}
		if rc != C.sp_return(C.SP_OK) {
			return error('Error ${int(rc)}. Failed to get port handler.')
		}
		return h
	}
} else {
	pub fn (this Port)handler() ?int {
		h := int(0)
		rc := unsafe {
			C.sp_get_port_handle(this.ptr, &h)
		}
		if rc != C.sp_return(C.SP_OK) {
			return error('Error ${int(rc)}. Failed to get port handler.')
		}
		return h
	}
}

fn C.sp_set_baudrate(port &C.sp_port, baudrate int) C.sp_return
pub fn (mut this Port)set_baudrate(baudrate int) bool {
	rc := C.sp_set_baudrate(this.ptr, baudrate)
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_set_bits(port &C.sp_port, bits int) C.sp_return
pub fn (this Port)set_bits(bits int) bool {
	rc := C.sp_set_bits(this.ptr, bits)
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_set_parity(port &C.sp_port, parity C.sp_parity) C.sp_return
pub fn (this Port)set_parity(parity Parity) bool {
	rc := C.sp_set_parity(this.ptr, C.sp_parity(parity))
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_set_stopbits(port &C.sp_port, stopbits int) C.sp_return
pub fn (this Port)set_stopbits(stopbits int) bool {
	rc := C.sp_set_stopbits(this.ptr, stopbits)
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_set_rts(port &C.sp_port, rts C.sp_rts) C.sp_return
pub fn (this Port)set_rts(rts Rts) bool {
	rc := C.sp_set_rts(this.ptr, C.sp_rts(rts))
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_set_cts(port &C.sp_port, cts C.sp_cts) C.sp_return
pub fn (this Port)set_cts(cts Cts) bool {
	rc := C.sp_set_cts(this.ptr, C.sp_cts(cts))
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_set_dtr(port &C.sp_port, dtr C.sp_dtr) C.sp_return
pub fn (this Port)set_dtr(dtr Dtr) bool {
	rc := C.sp_set_dtr(this.ptr, C.sp_dtr(dtr))
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_set_dsr(port &C.sp_port, dsr C.sp_dsr) C.sp_return
pub fn (this Port)set_dsr(dsr Dsr) bool {
	rc := C.sp_set_dsr(this.ptr, C.sp_dsr(dsr))
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_set_xon_xoff(port &C.sp_port, xon_xoff C.sp_xonxoff) C.sp_return
pub fn (this Port)set_xon_xoff(xon_xoff XonXoff) bool {
	rc := C.sp_set_xon_xoff(this.ptr, XonXoff(xon_xoff))
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_set_flowcontrol(port &C.sp_port, flowcontrol C.sp_flowcontrol) C.sp_return
pub fn (this Port)set_flowcontrol(flow_control FlowControl) bool {
	rc := C.sp_set_flowcontrol(this.ptr, C.sp_flowcontrol(flow_control))
	return rc == C.sp_return(C.SP_OK)
}

pub struct ReaderMode{
	blocking    = 0
	next        = 1
	nonblocking = 2
}

fn C.sp_blocking_read(port &C.sp_port, buf voidptr, count C.size_t, timeout_ms u32) C.sp_return
fn C.sp_blocking_read_next(port &C.sp_port, buf voidptr, count C.size_t, timeout_ms u32) C.sp_return
fn C.sp_nonblocking_read(port &C.sp_port, buf voidptr, count C.size_t) C.sp_return

pub fn (this Port)read(mode ReaderMode, max_length u32, timeout_ms u32) []byte {
	mut rc := int(0)
	if max_length == 0 {
		return 0
	}

	mut buff := []byte{cap: max_length}
	match mode {
		.blocking {
			rc = unsafe {
				int(C.sp_blocking_read(this.ptr, &buff[0], C.size_t(max_length), timeout_ms))
			}
			if rc < 0 {
				return []byte{}
			}
			buff.len = rc
			return buff
		}
		.next {
			rc = unsafe {
				int(C.sp_blocking_read_next(this.ptr, &buff[0], C.size_t(max_length), timeout_ms))
			}
			if rc < 0 {
				return []byte{}
			}
			buff.len = rc
			return buff
		}
		.nonblocking {
			rc = unsafe {
				int(C.sp_nonblocking_read(this.ptr, &buff[0], C.size_t(max_length)))
			}
			if rc < 0 {
				return []byte{}
			}
			buff.len = rc
			return buff
		}
	}
	return []byte{}
}

fn C.sp_blocking_write(port &C.sp_port, buf voidptr, count C.size_t, timeout_ms u32) C.sp_return
fn C.sp_nonblocking_write(port &C.sp_port, buf voidptr, count C.size_t) C.sp_return
pub fn (this Port)write(block bool, buffer []byte, timeout_ms u32) ?int {
	mut rc := int(0)

	if buffer.len == 0 {
		return error('Error: empty buffer.')
	}
	if block == true {
		rc = unsafe {
			int(C.sp_blocking_write(this.ptr, &buffer[0], buffer.len, timeout_ms))
		}
		if rc < 0 {
			return error('Error ${rc}. Failed to write.')
		}
		return rc
	}
	rc = unsafe {
		int(C.sp_nonblocking_write(this.ptr, &buffer[0], buffer.len))
	}
	if rc < 0 {
		return error('Error ${rc}. Failed to write.')
	}
	return rc
}


fn C.sp_input_waiting(port &C.sp_port) C.sp_return
pub fn (this Port)bytes_to_read() int {
	return int(C.sp_input_waiting(this.ptr))
}

fn C.sp_output_waiting(port &C.sp_port) C.sp_return
pub fn (this Port)bytes_to_write() int {
	return int(C.sp_output_waiting(this.ptr))
}

fn C.sp_flush(port &C.sp_port, buffers C.sp_buffer) C.sp_return
pub fn (this Port)flush(buffer Buffer) bool {
	rc := C.sp_flush(this.ptr, C.sp_buffer(buffer))
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_drain(port &C.sp_port) C.sp_return
pub fn (this Port)drain() bool {
	rc := C.sp_drain(this.ptr)
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_get_signals(port &C.sp_port, signal_mask &C.sp_signal) C.sp_return
pub fn (this Port)signal() ?Signal{
	csig := C.sp_signal(0)
	rc := unsafe {
		C.sp_get_signals(this.ptr, &csig)
	}

	if rc != C.sp_return(C.SP_OK) {
		errno := int(rc)
		return error('Error ${errno}. Failed to get signals.')
	}
	return Signal(csig)
}

fn C.sp_start_break(port &C.sp_port) C.sp_return
pub fn (this Port)start_break() bool {
	rc := C.sp_start_break()
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_end_break(port &C.sp_port) C.sp_return
pub fn (this Port)end_break() bool {
	rc := C.sp_end_break(this.ptr)
	return rc == C.sp_return(C.SP_OK)
}

//                
// Error handling
// 

fn C.sp_last_error_code() int
pub fn error_code() int {
	return C.sp_last_error_code()
}

fn C.sp_last_error_message() &char
pub fn error_message() string {
	cstr := unsafe {
		C.sp_last_error_message()
	}
	return cstr
}

fn C.sp_free_error_message(message &char)

fn C.sp_get_major_package_version() int
pub fn major_version() int {
	return C.sp_get_major_package_version()
}

//
// Version
// 

fn C.sp_get_minor_package_version() int
pub fn minor_version() int {
	return C.sp_get_minor_package_version()
}

fn C.sp_get_micro_package_version() int
pub fn micro_version() int {
	return C.sp_get_micro_package_version()
}

fn C.sp_get_package_version_string() &char
pub fn package_version() string {
	cstr := unsafe {
		cstring_to_vstring(C.sp_get_package_version_string())
	}
	return cstr
}

fn C.sp_get_current_lib_version() int
[inline]
pub fn current_lib_version() int {
	return C.sp_get_current_lib_version()
}

fn C.sp_get_revision_lib_version() int
[inline]
pub fn revision_lib_version() int {
	return C.sp_get_revision_lib_version()
}

fn C.sp_get_age_lib_version() int
[inline]
pub fn age_lib_version() int {
	return C.sp_get_age_lib_version()
}

fn C.sp_get_lib_version_string() &char
pub fn version() string {
	mut cstr := unsafe {
		cstring_to_vstring(C.sp_get_lib_version_string())
	}
	cstr = "libserialport: " + cstr
	return cstr
}

//
// PortConfiguration
//

struct C.sp_port_config{}
pub struct Configuration{
	ptr &C.sp_port_config
}

fn C.sp_new_config(&&C.sp_port_config) C.sp_return
pub fn new_configuration() ?Configuration {
	this := Configuration{}
	rc := unsafe {
		C.sp_new_config(&this.ptr)
	}
	if rc != C.sp_return(C.SP_OK) {
		return error('Failed to create new serial port Configuration')
	}
	return this
}

fn C.sp_free_config(&C.sp_port_config)
pub fn (this Configuration)free() {
	C.sp_free_config(this.ptr)
}

fn C.sp_get_config(port &C.sp_port, config &C.sp_port_config) C.sp_return
pub fn (this Configuration)get(port Port) bool {
	rc := C.sp_get_config(port.ptr, this.ptr)
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_set_config(port &C.sp_port, config &C.sp_port_config) C.sp_return
pub fn (this Configuration)set(port Port) bool {
	rc := C.sp_set_config(port.ptr, this.ptr)
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_get_config_baudrate(config &C.sp_port_config, baudrate_ptr &int) C.sp_return
pub fn (this Configuration)baudrate() int {
	br := int(0)
	unsafe {
		C.sp_get_config_baudrate(this.ptr, &br)
	}
	return br
}

fn C.sp_set_config_baudrate(config &C.sp_port_config, baudrate int) C.sp_return
pub fn (this Configuration)set_baudrate(baudrate int) bool {
	rc := C.sp_set_config_baudrate(this.ptr, baudrate)
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_get_config_bits(config &C.sp_port_config, bits_ptr &int) C.sp_return
pub fn (this Configuration)bits() int {
	bits := int(0)
	unsafe {
		C.sp_get_config_bits(thisa.ptr, &bits)
	}
	return bits
}

fn C.sp_set_config_bits(config &C.sp_port_config, bits int) C.sp_return
pub fn (this Configuration)set_bits(bits int) bool {
	rc := C.sp_set_config_bits(this.ptr, bits)
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_get_config_parity(config &C.sp_port_config, parity_ptr &C.sp_parity) C.sp_return
pub fn (this Configuration)parity() Parity {
	prt := C.sp_parity(0)
	unsafe {
		C.sp_get_config_parity(this.ptr, &ptr)
	}
	return Parity(ptr)
}

fn C.sp_set_config_parity(config &C.sp_port_config, parity C.sp_parity) C.sp_return
pub fn (this Configuration)set_parity(parity Parity) bool {
	rc := C.sp_set_config_parity(this.ptr, C.sp_parity(parity))
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_get_config_stopbits(config &C.sp_port_config, stopbits_ptr &int) C.sp_return
pub fn (this Configuration)stopbits() int {
	sb := int(0)
	unsafe {
		C.sp_get_config_stopbits(this.ptr, &sb)
	}
	return sb
}

fn C.sp_set_config_stopbits(config &C.sp_port_config, stopbits int) C.sp_return
pub fn (this Configuration)set_stopbits(stopbits int) bool {
	rc := C.sp_set_config_stopbits(this.ptr, stopbits)
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_get_config_rts(config &C.sp_port_config, rts_ptr &C.sp_rts) C.sp_return
pub fn (this Configuration)rts() Rts {
	rts := int(0)
	unsafe {
		C.sp_get_config_rts(this.ptr, &rts)
	}
	return Rts(rts)
}

fn C.sp_set_config_rts(config &C.sp_port_config, rts C.sp_rts) C.sp_return
pub fn (this Configuration)set_rts(rts Rts) bool {
	rc := C.sp_set_config_rts(this.ptr, C.sp_rts(rts))
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_get_config_cts(config &C.sp_port_config, cts_ptr &C.sp_cts) C.sp_return
pub fn (this Configuration)cts() Cts {
	cts := int(0)
	unsafe {
		C.sp_get_config_cts(this.ptr, &cts)
	}
	return Cts(cts)
}

fn C.sp_set_config_cts(config &C.sp_port_config, cts C.sp_cts) C.sp_return
pub fn (this Configuration)set_cts(cts Cts) bool {
	rc := C.sp_set_config_cts(this.ptr, C.sp_cts(cts))
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_get_config_dsr(config &C.sp_port_config, dsr_ptr &C.sp_dsr) C.sp_return
pub fn (this Configuration)dsr() Dsr {
	dsr := int(0)
	unsafe {
		C.sp_get_config_dsr(this.ptr, &dsr)
	}
	return Dsr(dsr)
}

fn C.sp_set_config_dsr(config &C.sp_port_config, dsr C.sp_dsr) C.sp_return
pub fn (this Configuration)set_dsr(dsr Dsr) bool {
	rc := C.sp_set_config_dsr(this.ptr, C.sp_dsr(dsr))
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_get_config_dtr(config &C.sp_port_config, dtr_ptr &C.sp_dtr) C.sp_return
pub fn (this Configuration)dtr() Dtr {
	dtr := int(0)
	unsafe {
		C.sp_get_config_dtr(this.ptr, &dtr)
	}
	return Dtr(dtr)
}

fn C.sp_set_config_dtr(config &C.sp_port_config, dtr C.sp_dtr) C.sp_return
pub fn (this Configuration)set_dtr(dtr Dtr) bool {
	rc := C.sp_set_config_dtr(this.ptr, C.sp_dtr(dtr))
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_get_config_xon_xoff(config &C.sp_port_config, xon_xoff_ptr &C.sp_xonxoff) C.sp_return
pub fn (this Configuration)xon_xoff() XonXoff {
	xx := int(0)
	unsafe {
		C.sp_get_config_xon_xoff(this.ptr, &xx)
	}
	return XonXoff(xx)
}

fn C.sp_set_config_xon_xoff(config &C.sp_port_config, xon_xoff &C.sp_xonxoff) C.sp_return
pub fn (this Configuration)set_xon_xoff(xon_xoff XonXoff) bool {
	rc := C.sp_set_config_xon_xoff(this.ptr, C.sp_xonxoff(xon_xoff))
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_set_config_flowcontrol(config &C.sp_port_config, flowcontrol C.sp_flowcontrol) C.sp_return
pub fn (this Configuration)(flow_control FlowControl) bool {
	rc := C.sp_set_config_flowcontrol(this.ptr, C.sp_flowcontrol(flow_control))
	return rc == C.sp_return(C.SP_OK)
}

//
// EventSet
//

struct C.sp_event_set {
	handles voidptr
	masks   &C.Event
	count   u32
}

pub struct EventSet {
	ptr &C.sp_event_set
}

fn C.sp_new_event_set(result_ptr &&C.sp_event_set) C.sp_return
pub fn new_event_set() ?EventSet{
	mut this := EventSet{}
	rc := unsafe {
		C.sp_new_event_set(&this.ptr)
	}
	if rc != C.sp_return(C.SP_OK) {
		return error('Error ${int(rc)}. Failed to create new EventSet.')
	}
	return this
}

fn C.sp_add_port_events(event_set &C.sp_event_set, port &C.sp_port, mask C.sp_event) C.sp_return
pub fn (this EventSet)add_port_events(port Port, event Event) bool {
	rc := C.sp_add_port_events(this.ptr, port.ptr, C.sp_event(event))
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_wait(event_set &C.sp_event_set, timeout_ms u32) C.sp_return
pub fn (this EventSet)wait(timeout_ms u32) bool {
	rc := C.sp_wait(this.ptr, timeout_ms)
	return rc == C.sp_return(C.SP_OK)
}

fn C.sp_free_event_set(event_set &C.sp_event_set)
pub fn (this EventSet)free() {
	C.sp_free_event_set(this.ptr)
}
