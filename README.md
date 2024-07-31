# CH32VX-GPIO-EXTI-PIO-Library
 Basic library for CH32V/X MCUs to enable digital I/O, trigger basic interrupt events with external pin stimulus and enable bit-banged I2C and SPI.

## Note
- The majority of this GPIO library is based on Larry Bank's work: https://github.com/bitbank2/CH32V_Experiments/tree/master?tab=readme-ov-file
    - My redistribution adds external interrupt functionality.
- The basic GPIO manipulation methods use a 16-Bit hex-scheme to represent the port/pin number (e.g 0x0C07 = Pin C7 = Pin 7 on Port C) (Port $\in$ [A,B,C,D])
- The provided example in main.c does not include any button debouncing logic. It is based on the WCH example: https://github.com/openwch/ch32v20x/tree/main/EVT/EXAM/EXTI/EXTI0/User
- Only applicable for CH32V20X: For the shortest response delay it is recommended to utilize External Interrupt on Pins, regardless of the Port, with the numbers being 0, 1, 2, 3, 4, 5 or 10, as Pins 5-9 and 10-15 each share their interrupt handlers.
    - For Interrupt Events on Pins 5 trough 9 and 10 through 15, each line is checked and the specified service routine is called successively, starting from the lowest number. So in case of multiple interrupts, happening simultaneously on multiple pins in the same handler group,
        each subroutine will be called in the order of their numbering, starting from the lowest number.
- Only applicable for CH32X03X: Interrupt Pins 0 through 7, 8 through 15 and 16 through 23 on every Port each share the same line or group and the specified service routine is called successively, starting from the lowest number. So in case of multiple interrupts, 
    happening simultaneously on multiple pins in the same handler group, each subroutine will be called in the order of their numbering, starting from the lowest number.

## Installation
### Prequisites
You need to have PlatformIO VSCode Plugin with the [WCH CH32V](https://github.com/Community-PIO-CH32V/platform-ch32v) Platform installed.
## Setup
Simply clone this repository onto your computer and open the folder like a regular PlatformIO project. You can try out the example in ```main.c```, if you have a CH32V203C8T6-EVT-R0 at hand.

Alternativly, you can also copy the library ```lib/CH32V_GPIO``` into any PIO project, modify your ```platformio.ini``` if needed and import the library into your ```main.c``` with: 
```c
#include "ch32v_gpio.h"
```

# Example

The example in this PIO project shows how to configure and interrupt function to be called when Pin A1 is pulled low (button pressed, negative edge). This will toggle an LED, connected to Pin A0 (externally provided).

## Supported MCUs
This library was only tested on the CH32V203C8T6-EVT-R0, but should work on any CH32V-family or CH32X-family of MCUs.

# Disclaimer

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.