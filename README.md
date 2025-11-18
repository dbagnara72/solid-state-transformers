# Solid state transformers

This repo contains a collection of simscape models concerning SST. 
**Remark** - add library to the path of matlab.

As known SST are build by a cascade of single-phase-inverter, where each one is galvanically insulated by DABs, LLCs and similar high efficient DC/DC converters.

The repo investigates on finding an optimal implementation in terms of efficiency and controllability.

Here a description of what folders contains.

**Remark** - each models implement a local time management on each DC/DC or DC/AC as well. Modulators generates trigger for the control system and the models permit 
to implement effects on local time sliding.

**theory analysis solid state transformer**:
Investigantion on different SST architectures:

- three phase DAB: very high efficiency on both semiconductor and magnetics, and high controllability, fundamental of the three phase at 4kHz;  
- single phase LLC: very high efficiency on semiconductor, magnetic run at 9.6kHz;  
- single phase DAB: high power loss on semiconductors;

- model implements two dab connected in parallel at battery side (1.5kV);
- each dab supplies a single phase inverter (400Vac);
- single phase inverters are connected in series (800Vac per phase);
- hw and sw implementation;
- n-independent time domains;

Improvements: LV DC grid at 800V, high voltage AC component at 3.3kV;

**theory analysis dab**:
- Two batteries connected among them by a DAB. This model is intended for deep DAB analysis in term of modulation strategies and efficiency analysis. The model contains a detailed Mosfet model.

**theory analysis resonant LLC**:
- Two batteries connected among them by an LLC. This model is intended for deep resonant-LLC analysis in term of modulation strategies and efficiency analysis. The model contains a detailed Mosfet model.

**single phase inverter**:
- Single phase inverter application with resonant pi, and system identification.
