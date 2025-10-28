# Lab 4
### EC535 - Embedded Systems Programming
### BeagleBone Black LED Blinking Kernel Module
Authors:
Yash Patel pately@bu.edu and
Leah Jones ldsj@bu.edu

Date: October 28, 2025
File: README.md


Submission File Structure Explanation:
- pately_ldsj_demo.mov: A video demonstration of the LED blinking functionality on the BeagleBone Black board.
- pately_ldsj_lab4/: The main directory containing the kernel module source code and Makefile.
  - Makefile: A file used to compile the kernel module.
  - mytraffic.c: The source code for the kernel module that controls the LED blinking behavior.
  - README: This file, providing an overview of the lab, implementation details, challenges faced


Contribution Statement: We contributed equally on this project. 


Implementation of the Lab: 
For this lab, we created a comprehensive traffic light controller kernel module for the BeagleBone Black. The module controls a 3-LED traffic light system with red, yellow, and green LEDs, implementing realistic traffic light behavior with multiple operational modes.

#### Key Features Implemented:
1. **Three Operational Modes:**
   - Normal Mode: Green (3 cycles) → Yellow (1 cycle) → Red (2 cycles), with pedestrian override
   - Flashing Red Mode: Continuous red light flashing (1 cycle on/off)
   - Flashing Yellow Mode: Continuous yellow light flashing (1 cycle on/off)
   - Lightbulb Check: When in any mode, you  press both buttons and all lights will flash, reseting it, and returning it to normal mode. 

2. **Button-Based Controls:**
   - BTN0 (GPIO 26): Cycles through operational modes with interrupt-driven debouncing
   - BTN1 (GPIO 46): Pedestrian call button that extends red phase to red+yellow for 5 cycles

3. **Character Device Interface (/dev/mytraffic):**
   - Reading provides current mode, cycle rate, LED status, and pedestrian status
   - Writing accepts integers 1-9 to set cycle rate in Hz (e.g., "3" = 3Hz = 333ms cycles)

4. **Thread Management:**
   - Dedicated kernel thread (traffic_thread) manages LED state machine
   - Spinlock-based synchronization for thread-safe operation between ISRs and main thread
   - Proper cleanup and resource management on module unload

The module uses GPIO pins 67 (red), 68 (yellow), and 69 (green) for LED control, with interrupt handlers for both button inputs. The implementation follows proper kernel development practices with error handling, resource cleanup, and thread-safe operations.

#### Challenges faced and how they were overcome:
1. **GPIO Pin Mapping and Configuration:** Understanding how to map BeagleBone Black physical pins to kernel GPIO numbers was initially challenging. We used the official pinout diagrams and BeagleBone documentation to correctly identify GPIO 26, 46, 67, 68, and 69 for our buttons and LEDs.

2. **Thread Synchronization:** Coordinating between interrupt handlers and the main traffic thread required careful consideration of race conditions. We initially used mutex locks but switched to spinlocks for better performance in interrupt context, ensuring thread-safe access to shared variables like `mode`, `cycle_rate_hz`, and `ped_waiting`.

3. **Interrupt Handling and Debouncing:** Button bouncing caused spurious mode changes and pedestrian requests. We implemented time-based debouncing using jiffies and a 200ms debounce period to filter out mechanical bounce effects.

4. **Hardware Circuit Assembly:** Wiring the traffic light circuit with proper current-limiting resistors and understanding active-low button behavior took careful attention to the provided circuit diagrams and BeagleBone electrical specifications.

5. **Kernel Thread Management:** Implementing a responsive kernel thread that could handle mode changes asynchronously while maintaining proper timing required careful use of `msleep()` and `kthread_should_stop()` checks.

We overcame these challenges through extensive use of kernel documentation, BeagleBone reference materials, and systematic debugging with `printk()` statements to trace execution flow and verify correct behavior.

#### References and Helpful Links Used:
-https://www.codeproject.com/articles/A-Novices-Guide-to-the-BeagleBone-Platform-and-Sof#comments-section 
-https://lwn.net/Kernel/LDD3/
-https://stackoverflow.com/questions
-https://docs.beagle.cc/intro/beagle101/blinkLED.html
