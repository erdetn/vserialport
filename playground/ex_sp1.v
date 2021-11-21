// Copyright(C) 2021 Erdet Nasufi. All rights reserved.

module main 

import vserialport

fn main() {
	mut p1 := vserialport.new_port('/dev/ttyUSB0') or {
		println('${err.msg}')
		return
	}

	p1.set_baudrate(38400)
	p1.set_bits(8)
	p1.set_parity(vserialport.Parity.@none)
	p1.set_stopbits(1)
	p1.set_flowcontrol(vserialport.FlowControl.@none)
	p1.set_xon_xoff(vserialport.XonXoff.disabled)
	p1.set_rts(vserialport.Rts.off)
	p1.set_cts(vserialport.Cts.ignore)
	p1.set_dtr(vserialport.Dtr.off)
	p1.set_dsr(vserialport.Dsr.ignore)
	
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


	p1.set_baudrate(9600)
	p1.set_parity(vserialport.Parity.@none)
	p1.set_bits(8)
	p1.set_stopbits(1)

	if p1.open(vserialport.Mode.read_write) == true {
		println('Port is open')
		p1.close()
	}

	code := vserialport.error_code()
	msg  := vserialport.error_message()
	println('>> [${code}]: ${msg}')


	p1.free()
}