# flipper_serprog

A **serprog-based SPI programmer** for use with [flashrom](https://flashrom.org/) on the **Flipper Zero**.

---

## Wiring Guide

| Flipper Zero Pin | Target Device Pin |
|------------------|-----------------|
| PA7              | MOSI            |
| PA6              | MISO            |
| PA4              | CS              |
| PB3              | SCK             |
| 3V3              | VCC             |
| GND              | GND             |

> **Note:** Some additional SPI chip pins may need to be held **high** for proper operation.  
> This can be done with **pull-up resistors** to 3V3, or (less recommended) by directly connecting them to the 3V3 rail.  
> Example: the `/HOLD` and `/WP` pins on Winbond chips.

> **Safety tip:**  
> If using a SOP8 clip, attach the device and verify connections **while the Flipper Zero is powered off**.  
> Voltage fluctuations on the 3V3 rail may interfere with other peripherals (e.g., the SD card interface) if the Flipper is powered on.

---

## Usage

Use **flashrom** to interface with the Flipper Zero via USB.

By default, `flipper_serprog` uses the **secondary VCP channel**, so the serprog interface may appear under a different `tty` device number.

You can change the VCP channel by modifying `USB_VCP_CHANNEL` in the source code and recompiling.

> If you set the VCP channel to `0`, the standard Flipper Zero RPC interface will be **unavailable** while `flipper_serprog` is running.

**Example usage:**

```bash
flashrom -p serprog:dev=/dev/tty.usbmodemflip_<flipper_name>3 -r spi.bin