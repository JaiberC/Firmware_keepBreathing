#!/bin/bash

cd
cd zephyrproject/build/zephyr

west build -b 96b_aerocore2 -p='always' ~/ruta/a/carpeta/de/proyecto
