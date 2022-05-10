#!/bin/bash
option=1
if [ $option -eq  0 ] 
then
	echo IP de la raspberry:
	read ip
	HOST=$ip
else
	HOST=192.168.2.3
fi

cd 
cd zephyrproject/build/zephyr

FILE=zephyr.elf
ALIAS=io.elf
RUTE="~/stm32f4/"
SCRIPT="cd stm32f4/; sudo ./script_programming.sh"
USERNAME=pi
PASSWORD=rpi123

echo Se enviara a la raspberry ${USERNAME} con la ip ${HOST} y la clave ${PASSWORD} el archivo ${FILE} con el alias ${ALIAS} en la ruta ${RUTE} y se le dara la orden ${SCRIPT}

sshpass -p ${PASSWORD} scp -P5300 ${FILE} ${USERNAME}@${HOST}:${RUTE}${ALIAS}

sshpass -p ${PASSWORD} ssh -p5300 ${USERNAME}@${HOST} "${SCRIPT}"
