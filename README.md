# What is this project about?
This is a project for the course "Embedded Systems Programming and Project" and is meant to teach us how to manage and participate in a project that is centered on SCRUM.
The appliance we chose to make for this project, is an automated care- and analysis system for autonomously managing the growth of various plants. Our goal is to create a modern plant pot,
that is outfitted with various sensors and actuators to monitor and take care of the seeds, and once those are grown, plants, that are planted in the pot.

# How do we achieve this?
The pot will come in the form-factor of a pod that has been fully sealed off from the outside, so that outside factors will not affect the plants' growth. From this point onwards, the 
plant-pot will be referred to as just a "pod".
Said pod will contain multiple sensors in order to measure the state of the plant it contains. The sensors that the pod will measure the following:
- Temperature
- Humidity of the air
- Humidity of the soil
- Pressure levels
- Light levels


Using these sensors, the following tools will be used to take care of the plants, and to display the current status of the plants:
- A waterpump
- A LCD screen


Utilising all of these sensors and actuators together, the pod will be able to autonomously take care of, and monitor, the plants it contains.

# What is this part of the project?
This branch is where it all comes together. The various sensors, communicative methods, and everything else will be merged here into one big program.

# How do you build the project?
This project can be built by opening it using Platformio on Visual Studio Code. Using Platformio, you can build and upload the code to the Nucleos. What part of the code needs to be uploaded (whether a Nucleo is a transmitter, or receiver, or something else) can be selected using [#define](/Src/main.c#L22) at the start of main.c.