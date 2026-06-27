# SR Actor

![ESP32](https://img.shields.io/badge/ESP32-grey?logo=Espressif)
![Relaise](https://img.shields.io/github/release-date/gerryvel/SR-Aktor?)
![lastcommit](https://img.shields.io/github/last-commit/gerryvel/SR-Aktor)
[![OBP](https://img.shields.io/badge/Sailing_with-OpenBoatsProjects-blue)](https://open-boat-projects.org/de/)

## Description
This repository shows how to switch 3 relais from Chartplotter with CZone Support and also with the website support.
- Switching Relais Aktor

and send it as NNMEA2000 meassage.
- PGN 127501 // Binary Status Report
- PGN 127502 // Binary Switch Control 
- PGN 126208 // Switch Control for B&G
- PGN 130817 
- PGN 65283
- PGN 65284
- PGN 65288
- PGN 65290 CZone  

In addition, all data and part of the configuration are displayed as a website. 

## Based on the work of

The project requires the NMEA2000 and the NMEA2000_esp32 libraries from Timo Lappalainen: https://github.com/ttlappalainen. 

This project is part of [OpenBoatProject](https://open-boat-projects.org/)

## Wiring diagram

<img width="3507" height="2480" alt="grafik" src="https://github.com/user-attachments/assets/e5de91e6-09d3-4055-8688-13e79fd537ea" />


## PCB Layout
PCB by Aisler: [Link](https://aisler.net/p/OFGLZJKF)

![grafik](https://github.com/user-attachments/assets/3358ef15-9ed8-4177-b05a-7ee3ab6d36b8)
![grafik](https://github.com/user-attachments/assets/60774f45-cd2a-4a4e-9cc4-e0a1b8fb4fe3)

## Website

<img width="1016" height="783" alt="grafik" src="https://github.com/user-attachments/assets/7e66f0bf-d480-4d03-8463-c04be6be1020" />

![image](https://github.com/user-attachments/assets/0d6cdb11-3c79-4dd9-9692-6c5469c97010)

![image](https://github.com/user-attachments/assets/0192a261-f228-4831-9c6f-785e468ea3c9)


## Chartplotter

I use an B&G Vulcan 7 with Czone Support, after import the file:

Configure Czone file with the CZone Configuration tool V 6.29.22

## Changes

- Version 1.3.2 CZone funtion with B&G Vulcan okay
- Version 1.0 Function with Website control, Monitor on NMEA200 okay (PGN127501)
