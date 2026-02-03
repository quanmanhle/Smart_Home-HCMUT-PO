# Smart_Home-HCMUT-PO


A team-based academic project developed at **Ho Chi Minh City University of Technology (HCMUT)**, focusing on **embedded systems**, **IoT communication**, and **secure system integration**.

---

## 📌 Overview
This project implements a **Smart Home IoT System** using **two ESP32 microcontroller nodes**, a **local Python-based gateway**, and the **CoreIoT cloud platform** (ThingsBoard-based).

The system supports real-time monitoring and control of home devices via MQTT, with additional **AI-based face recognition** for secure and contactless door access.

---

## 🏗 System Architecture

The system follows a **three-layer architecture**:

### 1️⃣ Edge Layer – ESP32 (FreeRTOS)
Two ESP32 nodes handle sensing and actuation:

- **Node 1 – Environmental Safety**
  - Gas detection (MQ-135) → exhaust fan control
  - Soil moisture monitoring → water pump control (dry-run protection)

- **Node 2 – Smart Access & Automation**
  - AI-controlled main door (servo motor)
  - Automatic garage door (ultrasonic sensor)
  - Smart lighting (LDR + WS2812B NeoPixel)
  - PIR-based fan control
  - Temperature & humidity sensing (DHT20)
  - LCD display via I2C

Each node:
- Runs on **FreeRTOS**
- Serializes sensor data as JSON
- Encrypts payloads using **AES-128**
- Publishes encrypted data via MQTT

---

### 2️⃣ Gateway Layer – Python Bridge
A local Python application acts as a secure gateway between edge devices and the cloud.

Responsibilities:
- Subscribe to encrypted MQTT topics from ESP32 nodes
- Decrypt payloads and validate data
- Forward telemetry to CoreIoT
- Route RPC control commands from cloud to local devices

This design offloads heavy computation and security handling from microcontrollers.

---

### 3️⃣ Cloud Layer – CoreIoT
The CoreIoT platform provides:
- Real-time telemetry dashboards
- Digital twin state management
- Remote device control via RPC
- Historical data visualization

---

## 🔐 Security Design
- **Encryption algorithm:** AES-128 (ECB mode)
- **Padding:** PKCS#7
- **Encoding:** Base64
- Encryption performed on ESP32 (C/C++)
- Decryption handled at gateway (Python – PyCryptodome)

Data remains encrypted within the local network and is only decrypted at the authorized gateway.

---

## 🤖 AI Face Recognition Module

The main door supports **AI-based face recognition** for access control.

### Processing Pipeline
1. Face detection using **Haar Cascade**
2. Grayscale image normalization (64×64)
3. Owner-only training dataset
4. Classification using **Isolation Forest**
5. Door unlock command published via MQTT

### Design Characteristics
- Local inference (no cloud dependency)
- Low latency and privacy-friendly
- Integrated directly into IoT control workflow


---
##🧪 Testing & Validation

Sensor functionality verified on ESP32 hardware

MQTT publish/subscribe stability tested under continuous operation

Encryption/decryption integrity validated

End-to-end data flow tested: ESP32 → Gateway → CoreIoT

Face recognition successfully triggered door actuation in real time

##🎥 Demo

A demonstration video showcasing:

Real-time sensor monitoring

Remote device control

AI-based face recognition unlocking the main door

▶️ YouTube Demo:
https://youtu.be/Mlss8x8UPN0
