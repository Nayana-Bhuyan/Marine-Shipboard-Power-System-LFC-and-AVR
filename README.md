# Marine-Shipboard-Power-System-LFC-and-AVR
This project simulates Load Frequency Control (LFC) and Automatic Voltage Regulation (AVR) for an isolated marine power system using MATLAB.
It demonstrates how frequency and voltage stability are achieved through PID controllers after a sudden load change in a shipboard electrical network.




Features
✔ Simulates frequency response and voltage response for a marine power system
✔ Uses PID controllers for LFC and AVR
✔ Dual Y-axis combined plot for better visualization
✔ Displays performance metrics:
  Frequency Settling Time
  Voltage Settling Time
  Overshoot values
   ✔ Saves output graph as PNG automatically




System Description
Marine Shipboard Power System

Components simulated:
Diesel generator (prime mover + governor dynamics)
AVR (Automatic Voltage Regulator)
Load Frequency Control (LFC) loop

Disturbance: Sudden load increase at t = 5s

Control Goal: Maintain 50 Hz and 1.0 p.u. voltage under load change




Simulation Details
Language: MATLAB
Method: Numerical simulation using time-step integration
Simulation Time: Default 30s
Disturbance: Step load increase
Controllers: PID for both frequency and voltage loops




Default Parameters
Nominal Frequency: 50 Hz
Nominal Voltage: 1.0 p.u.
Load Change Time: 5s
Simulation Time: 30s
PID Gains:
LFC: Kp=1.5, Ki=0.5, Kd=0.05
AVR: Kp=300, Ki=30, Kd=10




Performance Metrics Example
Frequency Settling Time	~12.5 s
Voltage Settling Time	~8.2 s
Frequency Overshoot	0.35 Hz
Voltage Overshoot	0.02 p.u.




Future Enhancements
Integrate Simulink model with visual block diagram
Implement nonlinear governor/turbine dynamics
Add different load scenarios
Use advanced control strategies (PI-FO, Fuzzy, etc.)




Applications
✔ Marine Electrical Systems
✔ Isolated Power Systems
✔ Power System Stability Studies

