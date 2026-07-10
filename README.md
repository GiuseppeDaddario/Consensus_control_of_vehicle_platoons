
# Consensus control of vehicle platoons
### Giuseppe D'Addario, Sapienza University of Rome, 2026

---
Implementation of consensus-based and safety-critical control algorithms for vehicle platooning [1].

### Contents
- `main.py`: interactive simulation entry point.
- `src/Controller.py`: control algorithms (e.g. bidirectional consensus, CBF safety filter).
- `src/Vehicle.py`: vehicle model and dynamics integration.
- `src/Viewer.py`: rendering and user interaction (pygame).
- `src/Simulation.py`: auxiliary simulation utilities (if present).
- `test/Tester.py` and `test/results/`: example scenarios and recorded outputs.

---

### Run
After installing the requirements you can run the pygame window with:
```bash
python3 main.py
```
Controls
- `W`: increase leader acceleration
- `S`: apply stronger deceleration to leader

---
### Tests and results
Demonstration videos are available under `test/results/`.

https://github.com/user-attachments/assets/441bd58b-f961-401e-aa5c-4a869700ec84

<b>Gap closing</b>: following vehicles close the gap from long distances while the virtual leader maintains a constant speed.

---
Other scenarios available:

- <b>Collision avoidance</b>: following vehicles approach the leading vehicle at high speeds.
- <b>Vehicle following</b>: all vehicles start at a standstill, and a reference acceleration is introduced to the virtual leader to reach a common velocity.
- <b>Platoon forming</b>: vehicles traveling at different speeds and positions coordinate to form a stable platoon.
- <b>Braking</b>: evaluates the string stability of the platoon (up to 10 vehicles) when the virtual leader performs a sudden braking maneuver.

---
### References
[[1]](https://www.sciencedirect.com/science/article/abs/pii/S0967066123002599) Ramzi Gaagai, Joachim Horn,
<b> Constrained distributed consensus control of homogeneous vehicle platoons with bidirectional communication </b>,
Control Engineering Practice,
Volume 140,
2023,
105690,
ISSN 0967-0661

