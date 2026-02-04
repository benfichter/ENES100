# Material Mermaids – Autonomous OTV (Object Transport Vehicle)

Autonomous Object Transport Vehicle (OTV) designed and built for a structured mission course involving navigation, obstacle avoidance, object pickup, weighing, and material identification.

![Final OTV Prototype](media/otv_final.jpg)
<!-- ↑ Replace with a clear photo of the completed robot -->

---

## Project Overview

The Material Mermaids OTV is a four-wheel autonomous robot capable of:
- Navigating a constrained course using ultrasonic sensing
- Traversing randomly placed obstacles
- Collecting a ball via a ramp/scoop intake
- Weighing the object using a load cell
- Classifying the object’s material based on weight and structure

The project emphasizes **mechanical design, propulsion modeling, embedded electronics, power budgeting, and system-level engineering documentation**.

---

## System Architecture

![System Block Diagram](media/system_block_diagram.png)
<!-- ↑ Optional: block diagram showing motors, sensors, MCU, battery -->

### Mechanical Design
- Plywood base chassis with four corner-mounted DC motors
- Direct-drive wheels with added surface treatment for traction
- Front ramp + scoop intake guiding object onto scale
- Load cell mounted between ramp and base for weighing

![Mechanical Layout](media/mechanical_layout.png)
<!-- ↑ Top-down or side view of the chassis -->

---

## Propulsion & Drive

- **Drive configuration:** 4-wheel differential drive  
- **Motors:** Greartisan 12V DC geared motors (200 RPM, high torque)  
- **Wheel radius:** 0.0762 m  
- **Steering:** Independent left/right motor speed control  

The drivetrain was analytically modeled for rolling resistance, torque requirements, and speed, then validated against real-world measurements.

![Torque-Speed Curve](media/torque_speed_curve.png)
<!-- ↑ Graph or free-body diagram -->

---

## Electronics & Power

### Major Components
- Arduino Mega (main controller)
- 2× L298N H-bridge motor drivers
- 4× HC-SR04 ultrasonic sensors
- HX711 + load cell (weight measurement)
- FSR402 force sensor
- WiFi module
- 12V Tenergy NiMH battery pack

![Wiring Diagram](media/wiring_diagram.png)
<!-- ↑ Clean wiring diagram or annotated photo -->

### Power Budget
- **Total current draw:** ~2.14 A
- **Battery:** 12V, 2000 mAh NiMH
- **Estimated runtime:** ~56 minutes

PWM motor control was used to limit current draw and improve traction reliability.

---

## Mission Capabilities

![Mission Course](media/mission_course.png)
<!-- ↑ Course layout or field photo -->

The OTV successfully:
- Navigates to within 150 mm of target locations
- Avoids and passes multiple obstacles
- Collects and weighs an object
- Classifies object type (foam vs rigid plastic)
- Completes the course autonomously

---

## Documentation

- **Milestone 9 – Final Design Briefs:** `docs/Milestone9_Final_Design_Briefs.pdf`
- **Sustainability / Eco Audit:** `docs/Sustainability_Eco_Audit_Report.pdf`
- **Full Engineering Report:** `docs/Report.pdf`

---

## Team

**Material Mermaids**  
Ava Mantzouranis  
Benjamin Fichter  
Nicholas Scozzafava  
Maren Iski  
Charlotte Grohowski  
Ethan Monders  
Alfonso Gruss  
Michael Baugh  

---

## Skills Demonstrated

- Mechanical system design
- Propulsion modeling and validation
- Embedded systems and motor control
- Sensor integration and data acquisition
- Power budgeting and sustainability analysis
- Technical documentation and engineering communication
