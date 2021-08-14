#!/bin/bash

cd
cd zephyrproject/build/zephyr

west build -b 96b_aerocore2 -p='always' ~/Documents/UniversidadNacional/ProyectoDeGrado/Firmware_keepBreathing/zephyr/08_controlador_final
