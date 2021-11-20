module main 

import time
import vserialport

fn main() {
	mut obd2 := vserialport.new_port('/dev/ttyUSB0') or {
		code := vserialport.error_code()
		msg  := vserialport.error_message()
		println('>> [${code}]: ${msg}')
		println('Failed to open:\n\t${err.msg}')
		return
	}
	
	obd2.set_baudrate(9600)
	obd2.set_bits(8)
	obd2.set_parity(vserialport.Parity.@none)
	obd2.set_stopbits(1)
	obd2.set_flowcontrol(vserialport.FlowControl.@none)
	obd2.set_xon_xoff(vserialport.XonXoff.disabled)
	
	match obd2.transport() {
		.native {
			println('Transport type: native.')
		}
		.usb {
			println('Transport type: usb')
			
			obd2_desc := obd2.description()
			ub := obd2.usb_bus().str()
			ud := obd2.usb_id().str()
			
			println('Desc:    ${obd2_desc}')
			println('USB Bus: ${ub}')
			println('USB ID:  ${ud}')
		}
		.bluetooth {
			println('Trasport type: bluetooth')
		}
	}

	println('obd2 port is connected = ${obd2.is_connected()}')

	mut rc := int(0)
	if obd2.open(vserialport.Mode.read_write) == false {
		code := vserialport.error_code()
		msg  := vserialport.error_message()
		println('>> [${code}]: ${msg}')
		obd2.free()
	}
	println('obd2 port is connected = ${obd2.is_connected()}')
	
	println('Port is open')

	rc = obd2.write(false, 'ATZ\r'.bytes(), 0) or {
		code := vserialport.error_code()
		msg  := vserialport.error_message()
		println('>> [${code}]: ${msg}')
		0
	}
	time.sleep(500*time.millisecond)
	println('${rc} bytes are written.')

	rc = obd2.write(false, '010D\r'.bytes(), 0) or {
		code := vserialport.error_code()
		msg  := vserialport.error_message()
		println('>> [${code}]: ${msg}')
		0
	}
	time.sleep(500*time.millisecond)
	msg1 := obd2.read(vserialport.ReaderMode.nonblocking, 10, 0)
	if msg1.len > 0 {
		dump(msg1)
	}

	time.sleep(500*time.millisecond)

	obd2.close()
	obd2.free()
}