// Copyright(C) 2021 Erdet Nasufi. All rights reserved.

module main 

import vserialport

fn main() {
	mut p1 := vserialport.new_port('/dev/ttyUSB1') or {
		code := vserialport.error_code()
		msg  := vserialport.error_message()
		println('>> [${code}]: ${msg}')
		println('Failed to open:\n\t${err.msg}')
		return
	}

	msg1 := [byte(0x03), 0x4A, 0x47, 0x40, 0x07, 0x42, 0x44, 0x64, 0xA9, 0x13]

	p1.set_baudrate(9600)
	p1.set_bits(8)
	p1.set_parity(vserialport.Parity.@none)
	p1.set_stopbits(1)
	p1.set_flowcontrol(vserialport.FlowControl.@none)
	p1.set_xon_xoff(vserialport.XonXoff.disabled)
	
	match p1.transport() {
		.native {
			println('Transport type: native.')
		}
		.usb {
			println('Transport type: usb')
			
			p1_desc := p1.description()
			ub := p1.usb_bus().str()
			ud := p1.usb_id().str()
			
			println('Desc:    ${p1_desc}')
			println('USB Bus: ${ub}')
			println('USB ID:  ${ud}')
		}
		.bluetooth {
			println('Trasport type: bluetooth')
		}
	}

	mut rc := 0
	if p1.open(vserialport.Mode.read_write) == true {
		println('Port is open')
		rc = p1.write(false, msg1, 0) or 
		{
			code := vserialport.error_code()
			msg  := vserialport.error_message()
			println('>> [${code}]: ${msg}')
			println('Failed to open:\n\t${err.msg}')
			0
		}
		println('${rc} bytes are written.')
		p1.close()
	}

	code := vserialport.error_code()
	msg  := vserialport.error_message()
	println('>> [${code}]: ${msg}')


	p1.free()
}