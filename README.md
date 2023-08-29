# Blinking Lights and Sound Controller

This Python script is designed to create an interactive system with blinking lights and sound effects, powered by a Raspberry Pi Pico microcontroller. Whether you're setting up an engaging light show or enhancing an event with synchronized sound effects, this script offers a captivating experience.

## Table of Contents

1. [Getting Started](#getting-started)
2. [Configuring Behavior](#configuring-behavior)
   - [Volume Control](#volume-control)
   - [LED Blinking](#led-blinking)
   - [Reaction Timeout](#reaction-timeout)
   - [Functionality Flags](#functionality-flags)
   - [Advanced I/O Configuration](#advanced-io-configuration)
3. [Using the Script](#using-the-script)
4. [Explore the Possibilities](#explore-the-possibilities)

## Getting Started

Before you begin, ensure you have:
- A Raspberry Pi Pico microcontroller.
- LEDs and a speaker connected to the Pico's pins.
- Properly configured UART communication with a DFPlayer module for sound playback.

## Configuring Behavior

### Volume Control
Adjust the `dfpVolume` parameter (ranging from 0 to 30) to set the volume level for the sound output. This enables you to match the sound intensity to your environment or preferences.

### LED Blinking
You have the flexibility to configure the LED blinking patterns for both the roof and front LEDs.

- **roofBlinkPatterns:** Specify an array to define the on and off times for the roof LEDs' blinking. You can adjust the duration of each phase of blinking to create captivating light effects.

- **frontBlinkPatterns:** Similar to roof LEDs, set an array for the front LEDs' blinking patterns. Tailor the on and off durations to achieve your desired visual impact.

### Reaction Timeout
The `reactionTimeout` parameter defines the maximum time in milliseconds that the system waits for an event. If no activity occurs within this timeframe, the system returns to its idle state. Adjust this setting based on how quickly you want the system to respond.

### Functionality Flags
Fine-tune the system's behavior using the `fConfig` dictionary, which contains several configuration flags:

- **dfpActive:** Enables or disables sound output in response to events.
- **dfpMultiple:** Determines whether multiple tracks can be played sequentially.
- **dfpRandom:** Enables random order playback when `dfpMultiple` is active.
- **keepAliveActive:** Activates a constant load to keep a power bank alive.
- **lightAlwaysActive:** Keeps the lights continuously active (sound added only on event if activated).

### Advanced I/O Configuration
This section allows you to configure inversion and pull-up/pull-down settings for different inputs and outputs:

- **invertRoofLeds** and **invertFrontLeds:** Inverts the LED outputs if needed.
- **invertKeepAlive:** Inverts the keep-alive output signal.
- **invertDips:** Inverts the dip switch inputs.
- **invertMoneyLed:** Inverts the money LED output.
- **invertMoneySensor:** Inverts the money sensor input.
- **invertDfpIO1:** Inverts the DFPlayer IO1 output.

## Using the Script

1. **Hardware Setup:** Power up your hardware components and Raspberry Pi Pico.

2. **Event-Based Actions:**
   - Input money or trigger a predefined event to activate blinking lights and sound effects.
   - The system automatically returns to a ready state after each event, awaiting the next input.

3. **Configuration Changes:**
   - Modify parameters in the script to tailor the system's behavior to your preferences.
   - Adjust volume, LED patterns, timeout, and other settings to achieve the desired effect.

4. **Exiting the Script:**
   - Terminate the script to stop the system.

## Explore the Possibilities

Explore the possibilities of creating mesmerizing light displays and immersive soundscapes with your Raspberry Pi Pico-driven system!
