# vserialport
This is a wrapper of [`libserialport`](https://sigrok.org/wiki/Libserialport) written in V. `vserialport` consists of the main datatype `Port`, and `Configuration`, `EventSet`, `UsbBus` and `UsbID` as secondary datatypes.


## Construct and deconstruct the serial `Port`
Create a `Port` object by calling `new_port`. `new_port` requires `port_name` argument as string (such as `devttyUSB0` or `/dev/ttyS0`) and returns optionally `Port` object. Use `or` to handle the error, in case it failes to create the `Port` object.
```V
mut p1 := vserialport.new_port('/dev/ttyUSB1') or {
	code := vserialport.error_code()
	msg  := vserialport.error_message()
	println('>> [${code}]: ${msg}')
	println('Failed to open:\n\t${err.msg}')
	return
}
```
In case the object is created, make sure to call the *destructor* function `free()` to deallocate the `Port` object. A safer approach is to use [defer](https://github.com/vlang/v/blob/master/doc/docs.md#defer).
```V
p1.free()
```

## Open and close the connection with serial port
Use `p1.open()` and `p1.close()` to open the connection with serial port and to disconnect the serial port. `open(mode)` requires `vserialport.Mode` argument and returns a boolean if it is connected or not. The other way to check if the `Port` is connected or not, use `p1.is_connected()` function.
```v
enum Mode {
	read
	write
	read_write
}
```
Such as:
```v
mut rc := int(0)
if p1.open(vserialport.Mode.read_write) == false {
	code := vserialport.error_code()
	msg  := vserialport.error_message()
	println('>> [${code}]: ${msg}')
	p1.free()
}
println('p1 port is connected = ${p1.is_connected()}')
```

## Read / write functions
`libserialport` supports non-blocking and blocking read/write functions. The same approach is reflected on `vserialport` as well. `write` function requires the first argument of boolean type to be set, if the write function is blocking or non-blocking write to serial port, respectively - that is, `block bool`: `true` for blocking write function, and `false` for non blocking write function. If it is non-blocking write function, the third argument - the `timeout_ms` argument is not going to be taken into consideration. It is useful only for blocking write function. The second argument is the byte array to be sent. In case you have to send `string`, `string` provides the `bytes()` function, which returns the byte array of that string. The `write` functions returns optional integer - that is, either number of bytes that are written to the serial port or the error that needs to be handled with `or` operator.

### Example 1:
```v
// Use non-blocking write to sent string AT+R
rc := p1.write(false, 'AT+R\r'.bytes(), 0) or {
	println('faild to write')
	return
}
println('${rc} bytes are sent.)
```
### Example 2
```v
// Use blocking write (and wait 1000 ms) to send this byte array [0x01, 0x31, 0x31, 0x04]
rc := p1.write(true, [byte(0x01), 0x31, 0x31, 0x04], 1000) or {
	println('faild to write')
	return
}
println('${rc} bytes are sent.)
```

Function prototype for read/write.
```v
write(block bool, buffer []byte, timeout_ms u32) ?int
read(mode ReaderMode, max_length u32, timeout_ms u32) []byte
```
In contrast with `write` function, the first argument of `read` function, instead of bool, requires `mode` as `enum ReaderMode` which has following three options:
```v
enum ReaderMode{
	blocking    = 0
	next        = 1
	nonblocking = 2
}
```


## Configuration functions
Use `set_baudrate()`, `set_bits()`, `set_parity()`, `set_stopbits()`, `set_rts(vserialport.Rts)`, `set_cts(vserialport.Cts)`, `set_dtr(vserialport.Dtr)`, `set_dsr(vserialport.Dsr)`, `set_xon_xoff(vserialport.XonXoff)` and `set_flowcontrol(vserialport.FlowControl)` to configure baudrate, bits, parity, stop bits, rts, cts, dtr, dsr, xon_xoff and flowcontrol, respectively.

Some other helping functions are: 
| function | return type | description |
| -- | -- | -- |
| `Port.name()` | `string` | returns the name of serial port.|
| `Port.description()`| `string` | returns the description of serial port. |
| `Port.transport()` | `enum Transport` | returns the transport type of serial port. `enum Transport {native, usb, bluetooth}`|
| `Port.usb_bus()` | `struct UsbBus` | returns the usb bus and usb address. |
| `Port.usb_id()` | `struct UsbId` | returns the USB vendor ID and USB product ID. |
| `Port.usb_manufacturer()` | `string` | returns the USB manufacturer. |
| `Port.usb_product()` | `string` | returns the USB product. |
| `Port.bluetooth_adress()` | `string` | returns the bluetooth address as string. |

## Other functions
|Function name | Return type | Description |
| -- | -- | -- |
| `Port.bytes_to_read()` | `int` | Return the number of bytes waiting in the input buffer. |
| `Port.bytes_to_write()` | `int` |  Return the number of bytes waiting in the output buffer. |
| `Port.flush(Buffer)` | `bool` | Flushes the `Buffer.input`, `Buffer.output` or `Buffer.both` and returns `true` if the selected buffer is flushed, otherwise `false`.|
| `Port.drain()` | `false` | Waits for the buffered data to be transmitted. | 
| `Port.signal()` | `enum Signal` | Returns the status of control signal. This depends from the control signals that the serial port is configured. This may trigger as output `Signal.cts`, `Signal.dsr`, `Signal.dcd` or `Signal.ri`. If it failes to get the control signal, error needs to be handled with `or` operator.|
| `Port.start_break` | `bool` | Puts the serial port transmission line into the break state. |
| `Port.end_break` | `bool` | Takes the serial port tranmission line out of the break state. |

Other helping function from `vserialport`:
|Function name | Return type | Description |
| -- | -- | -- |
| `error_code()` | `int` | Returns the last error code. |
| `error_message()` | `string` | Returns the last error message. |
| `major_version()` | `int` | Returns the major package version of `libserialport`. |
| `minor_version()` | `int` | Returns the minor package version of `libserialport`. |
| `micro_version()` | `int` | Returns the micro package version of `libserialport`. |
| `package_version()` | `string` | Returns the full package version of `libserialport` |

## Depedencies
- [libserialport](https://github.com/sigrokproject/libserialport)
## Bugfix
If you install the `libserialport` using `apt-get install libserialport*`, you may have to deal with following error: **"Inappropriate ioctl for device"**. To avoid this error, make sure to install the `libserialport` from the github [repository](https://github.com/sigrokproject/libserialport). More specifically, make sure that the following patch is applied ([patch](https://sigrok.org/bugzilla/attachment.cgi?id=733)):
```
configure.ac | 2 +-
1 file changed, 1 insertion(+), 1 deletion(-)

diff --git a/configure.ac b/configure.ac
index b1af16f..a26b851 100644
--- a/configure.ac
+++ b/configure.ac
@@ -112,7 +112,7 @@ AC_SYS_LARGEFILE
 AC_TYPE_SIZE_T
 
 # Check for specific termios structures.
-AC_CHECK_TYPES([struct termios2, struct termiox],,,
+AC_CHECK_TYPES([struct termios2],,,
 	[[#include <linux/termios.h>]])
 AC_CHECK_MEMBERS([struct termios.c_ispeed, struct termios.c_ospeed,
 		struct termios2.c_ispeed, struct termios2.c_ospeed],,,
```

## Roadmap
- [x] Writing the wrapper.
- [x] Testing main functions of `Port` struct.
- [ ] Testing other function of `Port` struct.
- [ ] Full testing of `Configuration` struct.
- [ ] Full testing of `EventSet` struct.
- [ ] Multithreading/concurrency.
- [ ] Testing on Windows OS.
- [ ] Testing on Linux OS.

## WARNING
This wrapper is not yet fully validated enough to be implemented in the application industry. 
