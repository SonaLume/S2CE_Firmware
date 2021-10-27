# S2CE_Firmware
Open source SonaLume Sound 2 Color Engine code for the SonaLume S2C Development Engine PCB

This project is licensed under the terms of the MIT license. Please see the LICENSE file for details...

This Microchip MPLABX based source code is intended for use in the SonaLume Sound 2 Color Engine Development Board currently available as version 0.6 at the SonaLume website. This device is a sound activated WS2812 LED controller. 

This firmware project works with the Sound 2 Color (S2C) Engine board to listen for sounds like music and speech which it then turns into various colors, similar to a color organ. Several unique algorithms are used to create interesting light shows depending upon which mode you choose to enable before compilation. Precompiled versions of the code for specific applications and LED panels will soon also be available for download.

The target microcontroller is a Microchip PIC 24FV32KA301 which is capable of driving up to 375 WS2812 LEDs. You will need to download the free Microchip MPLABX IDE system at https://www.microchip.com/en-us/development-tools-tools-and-software/mplab-x-ide as well as purchase an appropriate programmer to load the code onto the S2C Engine Development board. A Tag-Connect(TM) cable will also be needed https://www.tag-connect.com/debugger-cable-selection-installation-instructions depending on which programmer you choose. Both programmers and the proper Tag-Connect(TM) cables are available at the SonaLume website.

You will need to set up a new project in the MPLABX IDE, then after downloading or cloning this repository, you can add the various .c and .h files into the new project. After setting up the programmer and PIC part number in the IDE, you should be able to compile the code using the XC16 (v1.61) toolchain. At this point, you should be able to load the compiled hex file into a board and see it work. 

Different modes of operation can be selected in the main.c file that product different light effects. The source code is fairly well documented. Feel free to make the code better and submit your changes here for consideration. We plan on giving away some free S2C packages to the best and most popular modes that people create as well as to merge these new and better modes into the released code.

Different colors can also be created in the code, instead of the default ones.

New versions of the S2C Engine are in development and will be building upon the capabilities of this initial model. 

Our goal in making this code available to the public is to encourage people to make improvements to this base set of code for their own applications of the S2C Engine and to further the development of future S2C based projects and products. 

We are hoping to start an active community of S2C users and developers who can work together or individually to build upon this simple foundation and create even better light shows for the world to enjoy. We are doing our best to keep the cost of the hardware low so as to make it easier for everyone to become involved.



Safety warning:
During the development of this and other WS2812 products, we’ve become aware of some potentially hazardous issues with the wide variety of LED strips and panels available online as well as in some stores. Many of these products have large numbers of LEDs on them and these LEDs draw somewhere around 0.040 - 0.060 Amps of current each when all three (Red, Green and Blue) elements are fully driven on. We’ve measured more than 15 Amps of current being drawn by panels with 256 LEDs on them which is too much current for the (often low quality) power wires that come soldered to these panels, causing them to heat up to the point where the insulation begins to melt. Besides this, the panels themselves generate a lot of heat during operation. 

Most of our firmware uses only one or two of the elements of an individual LED at a time and only turns on a percentage of the total number of LEDs, both of which reduce the overall power consumption of the panel. We also install better quality wire onto our panels (when supplied as a kit) than they are usually shipped from the manufacturers with. These precautions, and the use of well designed and fully specified power supplies, represent our best efforts to ensure a safe and high quality product. However, we must caution our customers - as well as anyone looking to work with these kinds of LED products - to observe the following suggestions for their own safety:

Ensure the power supply is directly soldered to the LED panel’s DC inputs using the shortest lengths of properly specified, high quality stranded wiring possible. 

Verify using the wire manufacturer’s specifications that you are not overloading the wire. For a 256 LED panel, this could be more than 15 Amps at 5 VDC. 

Do not use barrel connectors to carry currents higher than 3 - 5 Amps. These connectors are not rated to carry a lot of current and heat up dangerously if you try to pass this much power through them. The best way is to make a permanent hard wired connection from the supply to the module. Second best would be to use a high current connector such as the XT60 used in RC model cars. 

Ensure there is enough ventilation for the LED panels and the power supply in your application. Not only can the parts get damaged by excessive heat, they can potentially start a fire. 

Do not “daisy-chain” panels using the three-pin JST connector’s power wiring to provide power to the next panel(s). The power and ground wires on most of these panels are short pieces of low quality 18AWG wires often with poorly attached terminations. They will overload and overheat when only one panel has multiple LEDs driven with all elements on (White). Asking this wire to handle even more power is dangerous. 

The SonaLume S2C Engine itself draws minimal current. It should be powered by the panel or strip’s JST signal cable and the panel or strip’s power and ground wires should go directly to the power supply’s output connections. 

For strips and panels with fewer LEDs, less power is consumed and less heat is generated. For instance, a 1 meter strip with 60 LEDs will draw a maximum of about 3.6 Amps at 5 VDC. This amount of current is near the limits of the low-quality factory wiring and barrel connectors usually found in plug in wall adapters. But, with carefully written firmware and display modes that only use individual elements (like Mode 0), you should be able to use the stock wires and barrel connectors. Heat and voltage drop can still be issues, but if you are careful you should be fine. 

Due to the nature of use and deployment of these modules, PCBs and accessories, Advanced Design and Manufacturing, LLC and its employees cannot be held responsible for any losses and/or damages related to the use of any and all products sold through our websites, directly or through distribution. 

