
# Particle Tracker Edge Firmware

Application reference firmware for the Particle Tracker.

# Resources

- [Latest Release](https://github.com/particle-iot/tracker-edge/releases)
- [Changelog](CHANGELOG.md)

### CREDITS AND ATTRIBUTIONS

The firmware uses the GNU GCC toolchain for ARM Cortex-M processors, standard peripheral libraries and Arduino's implementation of Wiring.

### LICENSE

Unless stated elsewhere, file headers or otherwise, all files herein are licensed under an Apache License, Version 2.0. For more information, please read the LICENSE file.

If you have questions about software licensing, please contact Particle [support](https://support.particle.io/).


### LICENSE FAQ

**This firmware is released under Apache License, Version 2.0, what does that mean for you?**

 * You may use this commercially to build applications for your devices!  You **DO NOT** need to distribute your object files or the source code of your application under Apache License.  Your source can be released under a non-Apache license.  Your source code belongs to you when you build an application using this reference firmware.

**When am I required to share my code?**

 * You are **NOT required** to share your application firmware binaries, source, or object files when linking against libraries or System Firmware licensed under LGPL.

**Why?**

 * This license allows businesses to confidently build firmware and make devices without risk to their intellectual property, while at the same time helping the community benefit from non-proprietary contributions to the shared reference firmware.

**Questions / Concerns?**

 * Particle intends for this firmware to be commercially useful and safe for our community of makers and enterprises.  Please [Contact Us](https://support.particle.io/) if you have any questions or concerns, or if you require special licensing.

_(Note!  This FAQ isn't meant to be legal advice, if you're unsure, please consult an attorney)_


### COMPILE & FLASH WITH WORKBENCH

This application must be built with device OS version 2.0.0-rc.3 and above.

1. Clone this repository `$ git clone git@github.com:particle-iot/tracker-edge.git && cd ./tracker-edge`
2. Init & Update Submodules `$ git submodule update --init --recursive`
3. Open Particle Workbench
4. Run the `Particle: Import Project` command, follow the prompts, and wait for the project to load
5. Run the `Particle: Configure Workspace for Device` command and select a compatible Device OS version and the `tracker` platform when prompted ([docs](https://docs.particle.io/tutorials/developer-tools/workbench/#cloud-build-and-flash))
6. Connect your device
7. Compile & Flash!

### CONTRIBUTE

Want to contribute to the Particle tracker edge firmware project? Follow [this link](CONTRIBUTING.md) to find out how.

### CONNECT

Having problems or have awesome suggestions? Connect with us [here.](https://community.particle.io/c/tracking-system).

Enterprise customers can contact [support](https://support.particle.io/).
