# SR Actor

![ESP32](https://img.shields.io/badge/ESP32-grey?logo=Espressif)
![Relaise](https://img.shields.io/github/release-date/gerryvel/SR-Aktor?)
![lastcommit](https://img.shields.io/github/last-commit/gerryvel/SR-Aktor)
[![OBP](https://img.shields.io/badge/Sailing_with-OpenBoatsProjects-blue)](https://open-boat-projects.org/de/)

## Description
This repository shows how to switch relais from Chartplotter with CZone Support 
- Switching Relais Aktor

and send it as NNMEA2000 meassage.
- PGN 127501 // Binary Status Report
- PGN 127502 // Binary Switch Control  

In addition, all data and part of the configuration are displayed as a website. 

## Based on the work of

The project requires the NMEA2000 and the NMEA2000_esp32 libraries from Timo Lappalainen: https://github.com/ttlappalainen. 

This project is part of [OpenBoatProject](https://open-boat-projects.org/)

## Wiring diagram

![grafik](https://github.com/user-attachments/assets/2530f91a-958b-4934-9f40-6623d9fc743e)

## PCB Layout

![grafik](https://github.com/user-attachments/assets/34d14d7f-d6ea-4557-a587-a4b4e61e0272)
![grafik](https://github.com/user-attachments/assets/f6119a11-2760-49e8-b35f-3bc5f063c6f3)
![grafik](https://github.com/user-attachments/assets/95d4d9c0-763f-456e-b900-4163e7338f9f)

## Website

![image](https://github.com/user-attachments/assets/2441b745-6de3-453a-989f-d68ef4060d7c)
![image](https://github.com/user-attachments/assets/0d6cdb11-3c79-4dd9-9692-6c5469c97010)
![image](https://github.com/user-attachments/assets/0192a261-f228-4831-9c6f-785e468ea3c9)


## Chartplotter

I use an B&G Vulcan 7 with Czone Support, after import the file:

![vulcan_v7_control](https://github.com/user-attachments/assets/014ecccb-5900-4c55-aafc-dfafc79efb1c)

Configure Czone file from the [Yacht Device Website](https://www.yachtd.com/products/ds/?czone)
