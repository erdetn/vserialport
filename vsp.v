// Copyright(C) 2021 Erdet Nasufi. All rights reserved.

module vsp

#include <libserialport.h>

#flag -I /usr/include
#flag -l serialport

pub enum Return { // C.sp _ return
	ok                = C.SP_OK
	invalid_arguments = C.SP_ERR_ARG
	fail 		  = C.SP_ERR_FAIL
	memory_error      = C.SP_ERR_MEM
	not_supported     = C.SP_ERR_SUPP
}

pub enum Mode { // Mode
	read       = C.SP_MODE_READ
	write      = C.SP_MODE_WRITE
	read_write = C.SP_MODE_READ_WRITE
}

pub enum Event { // Event
	received_ready = C.SP_EVENT_RX_READY
	transmit_ready = C.SP_EVENT_TX_READY
	event_error    = C.SP_EVENT_ERROR
}

pub enum Buffer { // Buffer
	input  = C.SP_BUF_INPUT
	output = C.SP_BUF_OUTPUT
	both   = C.SP_BUF_BOTH
}

pub enum Parity { // Parity
	invalid = C.SP_PARITY_INVALID
	@none   = C.SP_PARITY_NONE
	odd     = C.SP_PARITY_ODD
	event   = C.SP_PARITY_EVEN
	mark    = C.SP_PARITY_MARK
	space   = C.SP_PARITY_SPACE
}

pub enum Rts { // Rts
	invalid      = C.SP_RTS_INVALID
	off          = C.SP_RTS_OFF
	on           = C.SP_RTS_ON
	flow_control = C.SP_RTS_FLOW_CONTROL
}

pub enum Cts { // Cts
	invalide     = C.SP_CTS_INVALID
	ignore       = C.SP_CTS_IGNORE
	flow_control = C.SP_CTS_FLOW_CONTROL
}

pub enum Dtr { // Dtr
	invalide     = C.SP_DTR_INVALID
	off          = C.SP_DTR_OFF
	on           = C.SP_DTR_ON
	flow_control = C.SP_DTR_FLOW_CONTROL
}

pub enum Dsr { // Dsr
	invalide     = C.SP_DSR_INVALID
	ignore       = C.SP_DSR_IGNORE
	flow_control = C.SP_DSR_FLOW_CONTROL
}

pub enum XonXoff { // XonXoff
	invalide    = C.SP_XONXOFF_INVALID
	disabled    = C.SP_XONXOFF_DISABLED
	@in	    = C.SP_XONXOFF_IN
	out 	    = C.SP_XONXOFF_OUT
	inout 	    = C.SP_XONXOFF_INOUT
}

pub enum FlowControl { // FlowControl
	@none     = C.SP_FLOWCONTROL_NONE
	xon_xoff  = C.SP_FLOWCONTROL_XONXOFF
	rts_cts   = C.SP_FLOWCONTROL_RTSCTS
	dtr_dsr   = C.SP_FLOWCONTROL_DTRDSR
}

pub enum Signal { // Signal
	cts   = C.SP_SIG_CTS
	dsr   = C.SP_SIG_DSR
	dcd   = C.SP_SIG_DCD
	ri    = C.SP_SIG_RI
}

pub enum Transport { // Transport
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

fn C.sp_get_port_by_name(portname &char, port_ptr &&C.sp_port) Return
pub fn new_port(port_name string) Port {
	mut this := Port{
		ptr: voidptr(0)
	}
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

// fn C.sp_list_ports(struct sp_port ***list_ptr)

fn C.sp_open(port &C.sp_port, flags Mode) Return
pub fn (this Port)open(flags Mode) bool {
	rc := C.sp_open(this.ptr, Mode(flags))
	return rc == Return(C.SP_OK)
}

fn C.sp_close(port &C.sp_port) Return
pub fn (this Port)close() bool {
	rc := C.sp_close(this.ptr)
	return rc == Return(C.SP_OK)
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
	return cpd
}

fn C.sp_get_port_transport(port &C.sp_port) Transport
pub fn (this Port)transport() Transport {
	pt := Transport(C.sp_get_port_transport(this.ptr))
	return pt
}

pub struct UsbBus {
	usb_bus     int
	usb_address int
}

fn C.sp_get_port_usb_bus_address(port &C.sp_port, usb_bus &int, usb_address &int) Return
pub fn (this Port)usb_bus() UsbBus {
	ub := int(0)
	ua := int(0)
	rc := unsafe {
		C.sp_get_port_usb_bus_address(this.ptr, &ub, &ua)
	}

	if rc != Return(C.SP_OK) {
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

fn C.sp_get_port_usb_vid_pid(port &C.sp_port, usb_vid &int, usb_pid &int) Return
pub fn (this Port)usb_id() UsbID {
	vid := int(0)
	pid := int(0)
	rc := unsafe {
		C.sp_get_port_usb_vid_pid(this.ptr, &vid, &pid)
	}
	
	if rc != Return(C.SP_OK) {
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

fn C.sp_set_baudrate(port &C.sp_port, baudrate int) Return
pub fn (mut this Port)set_baudrate(baudrate int) bool {
	rc := C.sp_set_baudrate(this.ptr, baudrate)
	return rc == Return(C.SP_OK)
}

fn C.sp_set_bits(port &C.sp_port, bits int) Return
pub fn (this Port)set_bits(bits int) bool {
	rc := C.sp_set_bits(this.ptr, bits)
	return rc == Return(C.SP_OK)
}

fn C.sp_set_parity(port &C.sp_port, parity Parity) Return
pub fn (this Port)set_parity(parity Parity) bool {
	rc := C.sp_set_parity(this.ptr, Parity(parity))
	return rc == Return(C.SP_OK)
}

fn C.sp_set_stopbits(port &C.sp_port, stopbits int) Return
pub fn (this Port)set_stopbits(stopbits int) bool {
	rc := C.sp_set_stopbits(this.ptr, stopbits)
	return rc == Return(C.SP_OK)
}

fn C.sp_set_rts(port &C.sp_port, rts Rts) Return
pub fn (this Port)set_rts(rts Rts) bool {
	rc := C.sp_set_rts(this.ptr, Rts(rts))
	return rc == Return(C.SP_OK)
}

fn C.sp_set_cts(port &C.sp_port, cts Cts) Return
pub fn (this Port)set_cts(cts Cts) bool {
	rc := C.sp_set_cts(this.ptr, Cts(cts))
	return rc == Return(C.SP_OK)
}

fn C.sp_set_dtr(port &C.sp_port, dtr Dtr) Return
pub fn (this Port)set_dtr(dtr Dtr) bool {
	rc := C.sp_set_dtr(this.ptr, Dtr(dtr))
	return rc == Return(C.SP_OK)
}

fn C.sp_set_dsr(port &C.sp_port, dsr Dsr) Return
pub fn (this Port)set_dsr(dsr Dsr) bool {
	rc := C.sp_set_dsr(this.ptr, Dsr(dsr))
	return rc == Return(C.SP_OK)
}

fn C.sp_set_xon_xoff(port &C.sp_port, xon_xoff XonXoff) Return
pub fn (this Port)set_xon_xoff(xon_xoff XonXoff) bool {
	rc := C.sp_set_xon_xoff(this.ptr, XonXoff(xon_xoff))
	return rc == Return(C.SP_OK)
}

fn C.sp_set_flowcontrol(port &C.sp_port, flowcontrol FlowControl) Return
pub fn (this Port)set_flowcontrol(flow_control FlowControl) bool {
	rc := C.sp_set_flowcontrol(this.ptr, FlowControl(flow_control))
	return rc == Return(C.SP_OK)
}

pub enum ReaderMode{
	blocking    = 0
	next        = 1
	nonblocking = 2
}

fn C.sp_blocking_read(port &C.sp_port, buf voidptr, count C.size_t, timeout_ms u32) Return
fn C.sp_blocking_read_next(port &C.sp_port, buf voidptr, count C.size_t, timeout_ms u32) Return
fn C.sp_nonblocking_read(port &C.sp_port, buf voidptr, count C.size_t) Return

pub fn (this Port)read(mode ReaderMode, max_length u32, timeout_ms u32) []byte {
	mut rc := int(0)
	if max_length == 0 {
		return []byte{}
	}

	mut buff := []byte{cap: int(max_length)}
	match mode {
		.blocking {
			rc = unsafe {
				int(C.sp_blocking_read(this.ptr, &buff[0], C.size_t(max_length), timeout_ms))
			}
			if rc < 0 {
				return []byte{}
			}
			return buff
		}
		.next {
			rc = unsafe {
				int(C.sp_blocking_read_next(this.ptr, &buff[0], C.size_t(max_length), timeout_ms))
			}
			if rc < 0 {
				return []byte{}
			}
			return buff
		}
		.nonblocking {
			rc = unsafe {
				int(C.sp_nonblocking_read(this.ptr, &buff[0], C.size_t(max_length)))
			}
			if rc < 0 {
				return []byte{}
			}
			return buff
		}
	}
	return []byte{}
}

fn C.sp_blocking_write(port &C.sp_port, buf voidptr, count C.size_t, timeout_ms u32) Return
fn C.sp_nonblocking_write(port &C.sp_port, buf voidptr, count C.size_t) Return
pub fn (this Port)write(block bool, buffer []byte, timeout_ms u32) ?int {
	mut rc := int(0)

	if buffer.len == 0 {
		return error('Error: empty buffer.')
	}
	if block == true {
		rc = unsafe {
			int(C.sp_blocking_write(this.ptr, &buffer[0], C.size_t(buffer.len), timeout_ms))
		}
		if rc < 0 {
			return error('Error ${rc}. Failed to write.')
		}
		return rc
	}
	rc = unsafe {
		int(C.sp_nonblocking_write(this.ptr, &buffer[0], C.size_t(buffer.len)))
	}
	if rc < 0 {
		return error('Error ${rc}. Failed to write.')
	}
	return rc
}


fn C.sp_input_waiting(port &C.sp_port) Return
pub fn (this Port)bytes_to_read() int {
	return int(C.sp_input_waiting(this.ptr))
}

fn C.sp_output_waiting(port &C.sp_port) Return
pub fn (this Port)bytes_to_write() int {
	return int(C.sp_output_waiting(this.ptr))
}

fn C.sp_flush(port &C.sp_port, buffers Buffer) Return
pub fn (this Port)flush(buffer Buffer) bool {
	rc := C.sp_flush(this.ptr, Buffer(buffer))
	return rc == Return(C.SP_OK)
}

fn C.sp_drain(port &C.sp_port) Return
pub fn (this Port)drain() bool {
	rc := C.sp_drain(this.ptr)
	return rc == Return(C.SP_OK)
}

fn C.sp_get_signals(port &C.sp_port, signal_mask &Signal) Return
pub fn (this Port)signal() ?Signal{
	csig := Signal(0)
	rc := unsafe {
		C.sp_get_signals(this.ptr, &csig)
	}

	if rc != Return(C.SP_OK) {
		errno := int(rc)
		return error('Error ${errno}. Failed to get signals.')
	}
	return Signal(csig)
}

fn C.sp_start_break(port &C.sp_port) Return
pub fn (this Port)start_break() bool {
	rc := C.sp_start_break(this.ptr)
	return rc == Return(C.SP_OK)
}

fn C.sp_end_break(port &C.sp_port) Return
pub fn (this Port)end_break() bool {
	rc := C.sp_end_break(this.ptr)
	return rc == Return(C.SP_OK)
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
		cstring_to_vstring(C.sp_last_error_message())
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

fn C.sp_new_config(&&C.sp_port_config) Return
pub fn new_configuration() ?Configuration {
	this := Configuration{
		ptr: voidptr(0)
	}
	rc := unsafe {
		C.sp_new_config(&this.ptr)
	}
	if rc != Return(C.SP_OK) {
		return error('Failed to create new serial port Configuration')
	}
	return this
}

fn C.sp_free_config(&C.sp_port_config)
pub fn (mut this Configuration)free() {
	C.sp_free_config(this.ptr)
}

fn C.sp_get_config(port &C.sp_port, config &C.sp_port_config) Return
pub fn (this Configuration)get(port Port) bool {
	rc := C.sp_get_config(port.ptr, this.ptr)
	return rc == Return(C.SP_OK)
}

fn C.sp_set_config(port &C.sp_port, config &C.sp_port_config) Return
pub fn (this Configuration)set(port Port) bool {
	rc := C.sp_set_config(port.ptr, this.ptr)
	return rc == Return(C.SP_OK)
}

fn C.sp_get_config_baudrate(config &C.sp_port_config, baudrate_ptr &int) Return
pub fn (this Configuration)baudrate() int {
	br := int(0)
	unsafe {
		C.sp_get_config_baudrate(this.ptr, &br)
	}
	return br
}

fn C.sp_set_config_baudrate(config &C.sp_port_config, baudrate int) Return
pub fn (this Configuration)set_baudrate(baudrate int) bool {
	rc := C.sp_set_config_baudrate(this.ptr, baudrate)
	return rc == Return(C.SP_OK)
}

fn C.sp_get_config_bits(config &C.sp_port_config, bits_ptr &int) Return
pub fn (this Configuration)bits() int {
	bits := int(0)
	unsafe {
		C.sp_get_config_bits(this.ptr, &bits)
	}
	return bits
}

fn C.sp_set_config_bits(config &C.sp_port_config, bits int) Return
pub fn (this Configuration)set_bits(bits int) bool {
	rc := C.sp_set_config_bits(this.ptr, bits)
	return rc == Return(C.SP_OK)
}

fn C.sp_get_config_parity(config &C.sp_port_config, parity_ptr &Parity) Return
pub fn (this Configuration)parity() Parity {
	prt := Parity(0)
	unsafe {
		C.sp_get_config_parity(this.ptr, &prt)
	}
	return Parity(prt)
}

fn C.sp_set_config_parity(config &C.sp_port_config, parity Parity) Return
pub fn (this Configuration)set_parity(parity Parity) bool {
	rc := C.sp_set_config_parity(this.ptr, Parity(parity))
	return rc == Return(C.SP_OK)
}

fn C.sp_get_config_stopbits(config &C.sp_port_config, stopbits_ptr &int) Return
pub fn (this Configuration)stopbits() int {
	sb := int(0)
	unsafe {
		C.sp_get_config_stopbits(this.ptr, &sb)
	}
	return sb
}

fn C.sp_set_config_stopbits(config &C.sp_port_config, stopbits int) Return
pub fn (this Configuration)set_stopbits(stopbits int) bool {
	rc := C.sp_set_config_stopbits(this.ptr, stopbits)
	return rc == Return(C.SP_OK)
}

fn C.sp_get_config_rts(config &C.sp_port_config, rts_ptr &Rts) Return
pub fn (this Configuration)rts() Rts {
	rts := Rts(0)
	unsafe {
		C.sp_get_config_rts(this.ptr, &rts)
	}
	return Rts(rts)
}

fn C.sp_set_config_rts(config &C.sp_port_config, rts Rts) Return
pub fn (this Configuration)set_rts(rts Rts) bool {
	rc := C.sp_set_config_rts(this.ptr, Rts(rts))
	return rc == Return(C.SP_OK)
}

fn C.sp_get_config_cts(config &C.sp_port_config, cts_ptr &Cts) Return
pub fn (this Configuration)cts() Cts {
	cts := Cts(0)
	unsafe {
		C.sp_get_config_cts(this.ptr, &cts)
	}
	return Cts(cts)
}

fn C.sp_set_config_cts(config &C.sp_port_config, cts Cts) Return
pub fn (this Configuration)set_cts(cts Cts) bool {
	rc := C.sp_set_config_cts(this.ptr, Cts(cts))
	return rc == Return(C.SP_OK)
}

fn C.sp_get_config_dsr(config &C.sp_port_config, dsr_ptr &Dsr) Return
pub fn (this Configuration)dsr() Dsr {
	dsr := Dsr(0)
	unsafe {
		C.sp_get_config_dsr(this.ptr, &dsr)
	}
	return Dsr(dsr)
}

fn C.sp_set_config_dsr(config &C.sp_port_config, dsr Dsr) Return
pub fn (this Configuration)set_dsr(dsr Dsr) bool {
	rc := C.sp_set_config_dsr(this.ptr, Dsr(dsr))
	return rc == Return(C.SP_OK)
}

fn C.sp_get_config_dtr(config &C.sp_port_config, dtr_ptr &Dtr) Return
pub fn (this Configuration)dtr() Dtr {
	dtr := Dtr(0)
	unsafe {
		C.sp_get_config_dtr(this.ptr, &dtr)
	}
	return Dtr(dtr)
}

fn C.sp_set_config_dtr(config &C.sp_port_config, dtr Dtr) Return
pub fn (this Configuration)set_dtr(dtr Dtr) bool {
	rc := C.sp_set_config_dtr(this.ptr, Dtr(dtr))
	return rc == Return(C.SP_OK)
}

fn C.sp_get_config_xon_xoff(config &C.sp_port_config, xon_xoff_ptr &XonXoff) Return
pub fn (this Configuration)xon_xoff() XonXoff {
	xx := XonXoff(0)
	unsafe {
		C.sp_get_config_xon_xoff(this.ptr, &xx)
	}
	return XonXoff(xx)
}

fn C.sp_set_config_xon_xoff(config &C.sp_port_config, xon_xoff &XonXoff) Return
pub fn (this Configuration)set_xon_xoff(xon_xoff XonXoff) bool {
	rc := C.sp_set_config_xon_xoff(this.ptr, XonXoff(xon_xoff))
	return rc == Return(C.SP_OK)
}

fn C.sp_set_config_flowcontrol(config &C.sp_port_config, flowcontrol FlowControl) Return
pub fn (this Configuration)set_flowcontrol(flow_control FlowControl) bool {
	rc := C.sp_set_config_flowcontrol(this.ptr, FlowControl(flow_control))
	return rc == Return(C.SP_OK)
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

fn C.sp_new_event_set(result_ptr &&C.sp_event_set) Return
pub fn new_event_set() ?EventSet{
	mut this := EventSet{
		ptr: voidptr(0)
	}
	rc := unsafe {
		C.sp_new_event_set(&this.ptr)
	}
	if rc != Return(C.SP_OK) {
		return error('Error ${int(rc)}. Failed to create new EventSet.')
	}
	return this
}

fn C.sp_add_port_events(event_set &C.sp_event_set, port &C.sp_port, mask Event) Return
pub fn (this EventSet)add_port_events(port Port, event Event) bool {
	rc := C.sp_add_port_events(this.ptr, port.ptr, Event(event))
	return rc == Return(C.SP_OK)
}

fn C.sp_wait(event_set &C.sp_event_set, timeout_ms u32) Return
pub fn (this EventSet)wait(timeout_ms u32) bool {
	rc := C.sp_wait(this.ptr, timeout_ms)
	return rc == Return(C.SP_OK)
}

fn C.sp_free_event_set(event_set &C.sp_event_set)
pub fn (mut this EventSet)free() {
	C.sp_free_event_set(this.ptr)
}
