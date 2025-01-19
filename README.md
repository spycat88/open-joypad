# Open Joypad Device Tree Documentation

This document describes how to configure the **Open Joypad** driver via device tree. The driver supports four distinct operating modes:

- **none** (GPIO-only)
- **multiadc** (multiple ADC channels)
- **singleadc** (single ADC channel + analog multiplexer)
- **miyooserial** (Miyoo-like serial protocol over `/dev/ttyS1`)

Below you’ll find:

- [Overview](#overview)
- [Operating Modes](#operating-modes)
- [Properties](#properties)
  - [Required Properties](#required-properties)
  - [Optional Common Properties](#optional-common-properties)
  - [Multi-ADC Properties](#multi-adc-properties)
  - [Single-ADC (Multiplexer) Properties](#single-adc-multiplexer-properties)
  - [Miyoo Serial Properties](#miyoo-serial-properties)
- [Example Nodes](#example-nodes)
  - [1. GPIO-Only (No ADC)](#1-gpio-only-no-adc)
  - [2. Multi-ADC Mode](#2-multi-adc-mode)
  - [3. Single-ADC--Multiplexer](#3-single-adc--multiplexer)
  - [4. Miyoo-Serial Mode](#4-miyoo-serial-mode)
  - [5. Adding Rumble (PWM)](#5-adding-rumble-pwm)
- [Notes](#notes)

---

## Overview

The **Open Joypad driver** provides a flexible input subsystem interface for devices with various input configurations:

1. Simple **GPIO-only** button input (no analog).
2. **Multi-ADC** analog input, each axis mapped to its own ADC channel.
3. **Single-ADC** plus an **analog multiplexer** (amux), toggling between multiple axes on a single ADC input.
4. A **Miyoo serial** mechanism reading joystick frames over `/dev/ttyS1`.

Child nodes in the device tree define individual GPIO buttons, each with a `gpios` property and a `linux,code` specifying the Linux input keycode.

---

## Operating Modes

- **`none`**
  Only GPIO buttons, no analog axes.

- **`multiadc`**
  Multiple ADC channels, one per analog axis.

- **`singleadc`**
  A single ADC channel with an external analog multiplexer. The driver configures GPIO lines to select which axis to read.

- **`miyooserial`**
  Reads a 6-byte frame at 9600 8N1 on `/dev/ttyS1`, parsing two analog sticks (left & right) from the data.

---

## Properties

### Required Properties

- **`compatible`**
  Must be `"open,joypad"`.

- **`joypad-mode`**
  A string: `"none"`, `"multiadc"`, `"singleadc"`, or `"miyooserial"`.

- **Child nodes**:
  Each child node corresponds to a GPIO button.
  - `gpios` — `<&gpioX pinY GPIO_ACTIVE_LOW>` or similar.
  - `label` — optional human-readable name.
  - `linux,code` — integer, the Linux keycode (e.g. `0x131` for KEY_ENTER).
  - `linux,input-type` — optional; default `EV_KEY` if not specified.

### Optional Common Properties

- **`joypad-name`**
  String that overrides the default input device name (`"open-joypad"`).

- **`poll-interval`**
  Integer (milliseconds). How often the driver polls inputs (default `10`).

- **`autorepeat`**
  Boolean. If present, the driver enables auto-repeat for keys.

- **`button-adc-fuzz`**
  Integer specifying analog axis fuzz (noise filter). Defaults to `0`.

- **`button-adc-flat`**
  Integer specifying a flat zone for analog axes (± this range is zero). Defaults to `0`.

- **`button-adc-scale`**
  Integer scale factor for ADC values (default `1`).

- **`button-adc-deadzone`**
  Integer radial deadzone. If `sqrt(x^2 + y^2)` is below this, the axis is reported as zero.

- **Axis Inversion Flags**
  Each is a boolean property. If present, it inverts that axis:
  - `invert-absx`
  - `invert-absy`
  - `invert-absz`
  - `invert-absrx`
  - `invert-absry`
  - `invert-absrz`

- **`pwm-names`**
  If present (e.g. `"enable"`), the driver tries to get a PWM device for rumble. This enables a force-feedback interface (`FF_RUMBLE`).

### Multi-ADC Properties

*(Valid only if `joypad-mode = "multiadc"`)*

- **`io-channel-names`**
  A string array listing each ADC channel needed (e.g. `"adc-x", "adc-y", ..."`).

- **Axis Tuning Overrides**
  e.g. `abs_x-p-tuning`, `abs_x-n-tuning`, `abs_y-p-tuning`, etc. Each is an integer adjusting positive/negative scaling. Defaults to `180` if not set.

### Single-ADC (Multiplexer) Properties

*(Valid only if `joypad-mode = "singleadc"`)*

- **`amux-count`**
  The number of virtual channels (axes) read from the multiplexer (e.g. `4` for X/Y + RX/RY).

- **`amux-channel-mapping`**
  An array of 4 integers (default `<0 1 2 3>`) specifying how the multiplexer channels map to axes.

- **`amux-a-gpios`, `amux-b-gpios`, `amux-en-gpios`**
  GPIO lines for the multiplexer select bits and enable.

- **`io-channel-names`**
  Typically a single entry `"amux_adc"` referencing the single ADC input used by the AMUX.

- **Tuning Overrides**
  Similar to multi-ADC, e.g. `abs_x-p-tuning`, `abs_y-n-tuning`, etc.

### Miyoo Serial Properties

*(Valid only if `joypad-mode = "miyooserial"`)*

- The driver reads a 6-byte frame:
[0xFF, leftY, leftX, rightY, rightX, 0xFE]

- It automatically calibrates zero from ~50 frames on initialization.
- No additional DT properties required unless customizing calibration or the UART device node.

---

## Example Nodes

### 1. GPIO-Only (No ADC)

```dts
joypad0: joypad@0 {
  compatible = "open,joypad";
  joypad-mode = "none";
  poll-interval = <10>;
  autorepeat; /* optional boolean */

  button@0 {
      gpios = <&gpio0 5 GPIO_ACTIVE_LOW>;
      label = "BUTTON_A";
      linux,code = <0x131>;      /* e.g. KEY_ENTER */
      linux,input-type = <0x01>; /* EV_KEY */
  };

  button@1 {
      gpios = <&gpio0 6 GPIO_ACTIVE_LOW>;
      label = "BUTTON_B";
      linux,code = <0x133>;      /* e.g. KEY_HOME */
      linux,input-type = <0x01>; /* EV_KEY */
  };
};
```

### 2. Multi-ADC Mode

```dts
joypad1: joypad@1 {
    compatible = "open,joypad";
    joypad-mode = "multiadc";
    poll-interval = <10>;

    /* Provide multiple ADC channels for axes: e.g. RY, RX, Y, X, Z, RZ */
    io-channel-names = "adc-ry", "adc-rx", "adc-y", "adc-x", "adc-z", "adc-rz";
    button-adc-scale = <1>;
    button-adc-deadzone = <200>;

    /* Example: invert right-X axis */
    invert-absrx;

    /* Optional GPIO buttons as child nodes */
    button@0 {
        gpios = <&gpio1 12 GPIO_ACTIVE_LOW>;
        label = "SELECT_BUTTON";
        linux,code = <0x161>;
    };
    button@1 {
        gpios = <&gpio1 13 GPIO_ACTIVE_LOW>;
        label = "START_BUTTON";
        linux,code = <0x162>;
    };
};
```

### 3. Single-ADC + Multiplexer

```dts
joypad2: joypad@2 {
    compatible = "open,joypad";
    joypad-mode = "singleadc";
    poll-interval = <10>;
    amux-count = <4>;

    /* Single ADC channel for the mux output */
    io-channel-names = "amux_adc";
    amux_adc: io-channels = <&saradc0 0>;

    /* MUX selection lines */
    amux-a-gpios = <&gpio0 14 GPIO_ACTIVE_LOW>;
    amux-b-gpios = <&gpio0 15 GPIO_ACTIVE_LOW>;
    amux-en-gpios = <&gpio0 16 GPIO_ACTIVE_LOW>;

    /* Map internal mux channels; default is <0 1 2 3> */
    amux-channel-mapping = <0 1 2 3>;

    /* Example additional GPIO buttons */
    button@0 {
        gpios = <&gpio2 10 GPIO_ACTIVE_LOW>;
        label = "BUTTON_X";
        linux,code = <0x133>;
    };
    button@1 {
        gpios = <&gpio2 11 GPIO_ACTIVE_LOW>;
        label = "BUTTON_Y";
        linux,code = <0x134>;
    };
};
```

### 4. Miyoo-Serial Mode

```dts
joypad3: joypad@3 {
    compatible = "open,joypad";
    joypad-mode = "miyooserial";
    poll-interval = <10>;

    /* The driver opens /dev/ttyS1 at 9600 8N1 to parse frames */

    button@0 {
        gpios = <&gpio3 22 GPIO_ACTIVE_LOW>;
        label = "A_BUTTON";
        linux,code = <0x130>;
    };
    button@1 {
        gpios = <&gpio3 23 GPIO_ACTIVE_LOW>;
        label = "B_BUTTON";
        linux,code = <0x131>;
    };
};
```

### 5. Adding Rumble (PWM)

```dts
joypad-rumble: joypad@4 {
    compatible = "open,joypad";
    joypad-mode = "none";

    /* Indicate a PWM device for vibration/rumble */
    pwm-names = "enable";
    /* Reference a valid PWM in your SoC */
    pwms = <&pwm0 0 1000000 0>;

    button@0 {
        gpios = <&gpio0 17 GPIO_ACTIVE_LOW>;
        label = "BUTTON_L";
        linux,code = <0x136>;
    };
};
```

---

## Notes

1. **GPIO References**
Your SoC may define GPIO pins differently (e.g. `<&pio 0 0 GPIO_ACTIVE_HIGH>`). Adjust accordingly.

2. **Keycodes**
`linux,code` must be a valid Linux input keycode (see `include/uapi/linux/input-event-codes.h`).

3. **ADC Requirements**
For `multiadc` or `singleadc`, ensure you have matching `io-channels` defined for your SoC or external ADC. Each name in `io-channel-names` must match a channel resource in the device tree.

4. **Serial**
The Miyoo serial mode is currently hardcoded to read `/dev/ttyS1` at 9600 8N1. Modify the driver source if you need a different UART or baud rate.

5. **Rumble/Force Feedback**
Specifying `pwm-names = "enable"` tells the driver to register a force-feedback device (`FF_RUMBLE`) using the PWM. From user space, you can test or trigger rumble via typical FF APIs or tools such as `fftest`.
