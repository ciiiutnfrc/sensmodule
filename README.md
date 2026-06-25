# Módulo sensor de código abierto para robot móvil

El módulo de sensores consta de dos componentes principales:
1. un anillo de sensores de ultrasonido o sonar (Sonar NAvigation and Ranging) para medición de distancia
1. una unidad de medición inercial, que integra un acelerómetro y un giroscopio (IMU, Inertial Measurement Unit)

Para sujetar y distribuir los transductores de ultrasonido como anillo circular se diseñó un soporte mecánico especı́fico. Todo el conjunto se aloja en un gabinete dedicado fabricado con impresión 3D. El diseño electrónico fue realizado utilizando [KiCAD](https://www.kicad.org/), mientras que para el diseño mecánico del soporte de los transductores y el gabinete se utilizó [FreeCAD](https://www.freecad.org/); ambos decódigo abierto.

Está implementado en base a la placa de desarrollo ["Raspberry Pi Pico"](https://www.raspberrypi.com/documentation/microcontrollers/pico-series.html#pico1), cuyo firmware actúa como nodo de [ROS](https://www.ros.org/) utilizando el framework [micro-ROS](https://micro.vulcanexus.org/). Este nodo es responsable de adquirir los datos de todos los sensores y publicarlos en los topics correspondientes para su procesamiento en la computadora principal del robot.

## Videos
* [ModSens - sensores de distancia por ultrasonido](https://www.youtube.com/watch?v=mX4Z-RGdCro)
* [ModSens - visualización RViz sobre el robot RoMAA-II](https://www.youtube.com/watch?v=sf0R5RFJkfc)
* [ModSens - sensores inerciales](https://www.youtube.com/watch?v=RdyZEUkLeZU)
