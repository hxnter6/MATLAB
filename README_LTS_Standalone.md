# LTS (Lap Time Simulation) Standalone System

## Overview

This is a comprehensive standalone MATLAB file containing all the functionality of the LTS (Lap Time Simulation) system for Formula SAE/FSG vehicles. It consolidates all the original functionality into a single, independent file that can be run without requiring any additional dependencies.

## Features

- **Complete Vehicle Dynamics**: Cornering, acceleration, and braking simulation
- **Pacejka '93 Tire Model**: Full implementation with lateral and longitudinal tire modeling
- **Engine Modeling**: Force curve processing from engine data
- **DRS Functionality**: Drag Reduction System with intelligent activation
- **Track Simulation**: Segment-by-segment lap time calculation
- **FSG Points Calculation**: Based on 2018 Formula Student Germany rules
- **Comprehensive Output**: Detailed plots and simulation reports

## Files

- `LTS_Standalone.m`: The main standalone simulation file
- `test_standalone.m`: Simple test script to validate functionality
- `README_LTS_Standalone.md`: This documentation file

## Usage

1. **Simple Usage**: Just run `LTS_Standalone.m` in MATLAB
2. **Testing**: Run `test_standalone.m` to verify the system works correctly
3. **Customization**: Edit the vehicle parameters, track data, and engine data sections to customize for your application

## Key Components

### Vehicle Parameters
- Mass properties (driver, car, accumulator, DRS)
- Dimensions (track width, wheelbase, CG height)
- Aerodynamics (downforce, drag, frontal area)
- DRS settings (drag reduction, downforce loss)

### Tire Model
- Hoosier R25B tire coefficients for lateral and longitudinal forces
- Pacejka '93 model implementation
- Grip modification factors
- Slip limits and resolution settings

### Engine Data
- Sample Emrax 208 HV motor data
- Power and torque curves
- Gear ratios and shifting points
- RPM limits and power scaling

### Track Data
- Sample Ice Cream Cone 2 track layout
- Segment radius and distance data
- Easy to modify for different tracks

## Output

The simulation produces:
- **Lap time**: Total time to complete one lap
- **Average speed**: Mean speed around the track
- **Performance plots**: Speed vs time, DRS activation, slip angles
- **Simulation report**: Complete data structure with all results
- **FSG points**: Calculated according to FSG rules

## Customization

### Changing Vehicle Parameters
Edit the "VEHICLE PARAMETERS" section to modify:
- Vehicle masses and dimensions
- Aerodynamic characteristics
- DRS settings
- Engine parameters

### Changing Track
Edit the "SAMPLE TRACK DATA" section to define your track:
- `r`: Array of corner radii (positive = left turn, negative = right turn, 0 = straight)
- `d`: Array of segment distances in meters

### Changing Engine
Edit the "SAMPLE ENGINE DATA" section:
- Replace the engine_data matrix with your engine's RPM, power, torque, fuel flow
- Update gear ratios and shifting speeds

## Technical Details

### Simulation Flow
1. **Vehicle Model**: Precomputes performance limits for DRS ON/OFF states
2. **Engine Processing**: Converts engine data to force curves
3. **Track Simulation**: Simulates each track segment optimizing speed
4. **Post-processing**: Generates plots and calculates final statistics

### Key Functions
- `pacejka_fun_93()`: Core tire model implementation
- `Vehicle_Sim_nlin()`: Vehicle performance limit calculation
- `Engine_Force_Curves()`: Engine data processing
- `Maneuver_Sim_fixed()`: Track simulation with DRS logic
- `Straight2()`: Straight-line acceleration/braking simulation
- `corner()`: Corner simulation with optimal speed calculation

### DRS Logic
DRS is activated when:
- Vehicle is on a straight segment
- Lateral G-force is below threshold (0.9g default)
- DRS is enabled in vehicle parameters

## Troubleshooting

### Common Issues
1. **Variable scope errors**: All variables are properly managed with persistent variables
2. **Missing functions**: Everything is contained in the single file
3. **Track data format**: Ensure r and d arrays have the same length
4. **Engine data format**: Check that engine data matrix has correct columns

### Performance Tips
- Reduce velocity range resolution (increase dv) for faster simulation
- Reduce tire model resolution (increase SA_res, SR_res) for faster calculation
- Use smaller time steps (reduce dt) for more accurate results

## Validation

The system has been tested to ensure:
- No syntax errors or undefined variables
- Proper function scoping and variable management
- Realistic simulation results
- Consistent output format
- Compatibility with MATLAB (tested structure)

## Original Code Credits

This standalone version is based on the original LTS system developed by SCoats, with all functionality consolidated and bugs fixed for independent operation.

## License

This code is provided as-is for educational and research purposes. Please refer to the original user agreement and guide for usage restrictions and terms.

---

**Quick Start**: Simply run `LTS_Standalone.m` in MATLAB to see the complete system in action!