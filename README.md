# EcoMAT: Permanent Magnet Based Asset Tracking


## Overview
Welcome to our repository, dedicated to advancing asset tracking in cleanroom environments through the innovative use of permanent magnets. Our project centers on a novel asset tracking system, named "EcoMAT," which harnesses the unique properties of permanent magnets to offer a cost-effective, highly accurate, and scalable solution for monitoring hazardous chemicals.


## Repository Structure
- **data**
  - Stores all datasets and experimental results organized by date and experiment type.
- **data_acquisition**
  - Contains tools and scripts used for collecting data from various devices and sensors.
- **data_analysis**
  - Includes Jupyter notebooks and other scripts for data processing, analysis, and visualization.
- **LICENSE**
  - Contains the terms under which the project can be used.
- **README.md**
  - Provides an overview of the project, setup instructions, and other necessary documentation to get started.

### Data
Contains datasets organized by experimental setup and date. Each subdirectory is named after the type of experiment or the date it was conducted, such as:
- **Multi-Magnet Tests**: `apr-20-multi-magnet-tests` includes data from experiments testing the effect of multiple magnets on localization accuracy.
- **Preliminary Localization Data**: Directories like `apr-4-prlm-localisation-data-each-position` contain data from initial tests aimed at determining the positional accuracy of the localization system.
- **Strength-Distance Correlation Data**: `mar-27-prlm-prm-prmm-strength-dist-corr-re-gather` gathers data to examine how the strength of the magnetic field varies with distance, essential for optimizing sensor placement.

### Data Acquisition
This directory houses all necessary software components for the initial data collection processes specific to magnetic field monitoring using STM32 microcontrollers. Here's a breakdown of its subdirectories and key files:

#### MaintStorage-magnetic-field-trial
This subdirectory contains the embedded software suite developed for STM32 microcontrollers, designed to capture and process magnetic field data from various sensors. The software is organized into several key components:

##### Core
This folder contains the primary source code that orchestrates the data acquisition process. It includes:
- **Inc**: Header files that define interfaces and configurations necessary for the operation of the microcontroller and connected sensors.
- **Src**: Source files that implement the logic for initializing the system, managing sensor data inputs, and processing those inputs into a usable format.
- **Startup**: Contains the startup assembly code which initializes the microcontroller, setting up essential services and runtime environment.

##### Debug
Contains compiled binaries and debugging files to assist in the development and troubleshooting of the firmware. This includes:
- **Core**: Compiled core functionality for quick testing and deployment.
- **Drivers**: Contains compiled driver modules that interface directly with the hardware components.
- **MaintStorage-magnetic-field-trial.elf**: The executable link file that can be loaded directly onto the microcontroller for execution.

##### Drivers
This folder includes all necessary driver files that facilitate communication between the microcontroller and its peripherals (such as sensors and actuators). Key components include:
- **CMSIS**: ARM's Cortex Microcontroller Software Interface Standard, providing the necessary low-level interface to the hardware.
- **STM32WBxx_HAL_Driver**: Hardware Abstraction Layer drivers for STM32WB series, simplifying the usage of device peripherals.

#### Usage Instructions
To utilize the software:
1. Navigate to the project directory in your terminal.
2. Compile the code using the provided Makefile:
   Follow the STM32 compilation standard.


### Data Analysis
This directory contains scripts and data files used for analyzing the data collected by the monitoring systems. It includes various Jupyter notebooks and CSV files for processing and visualizing data. Below are the subdirectories and their contents:

#### Notebooks
- **data-acquisition-pre-processing-plotting.ipynb**: Jupyter notebook for pre-processing and plotting the raw data collected during experiments.
- **multi-lateration-for-magnet-localisation.ipynb**: Contains algorithms and visualizations for multi-lateration techniques used in magnet localization studies.

#### Localisation Latency Analysis
This folder contains CSV files mapping the latency of localization results for different sensor configurations:
- **PRLM-1_latency-map.csv**: Latency mappings for the PRLM-1 sensor configuration.
- **PRM-1_latency-map.csv**: Latency mappings for the PRM-1 sensor configuration.
- **PRMM-1_latency-map.csv**: Latency mappings for the PRMM-1 sensor configuration.

#### Same-Type Similar-Strength Analysis
Includes CSV files for analysis of sensors of the same type under similar strength conditions to assess consistency and reliability:
- **PRM-1-compared-with-2-other-PRMs.csv**: Comparison of three PRM sensors to evaluate variance in sensor readings under controlled conditions.

#### Sensor Test
Contains text files with test results from different sensors under various experimental setups:
- **nov-9-drv425-prlm-stm32-distance-test.txt**: Test results for the DRV425 sensor in a distance measurement setup.
- **nov-9-ss496a1-prlm-stm32-test.txt**: Test results for the SS496A1 sensor under standard operation conditions.
- **oct-30-lis3mdl-prlm-stm32-distance-test.txt**: Distance test results for the LIS3MDL sensor.

#### X-Lateration Analysis
This subdirectory includes CSV files for lateration analysis, useful for verifying the accuracy of position estimates from multiple sensor data points:
- **PRLM-1_3-LATERATION-map.csv**: Three-point lateration mapping for PRLM-1.
- **PRLM-1_4-LATERATION-map.csv**: Four-point lateration mapping for PRLM-1.
- **PRM-1_3-LATERATION-map.csv**, **PRM-1_4-LATERATION-map.csv**, **PRM-1_5-LATERATION-map.csv**: Lateration mappings for PRM-1 with varying numbers of data points.
- **PRMM-1_3-LATERATION-map.csv**, **PRMM-1_4-LATERATION-map.csv**, **PRMM-1_5-LATERATION-map.csv**: Lateration mappings for PRMM-1 configurations.

#### Usage Instructions
To analyze the collected data:
1. Navigate to the `data_analysis` directory.
2. Open the Jupyter notebooks in a Jupyter environment to run the analysis:
   ```bash
   jupyter notebook data-acquisition-pre-processing-plotting.ipynb


## Getting Started
### Prerequisites
Ensure you have Python and Jupyter installed to run the analysis notebooks. 

### Installation
Clone the repository and install the necessary dependencies:
```bash
git clone [repository-url]
cd [project-directory]
pip install -r requirements.txt
```

### Running the Tools
Navigate to the `data_analysis` directory to start the Jupyter Notebooks:
```bash
cd data_analysis
jupyter notebook
```

## How to Contribute
Contributions to improve the functionality or extend the capabilities of the monitoring systems are welcome. Please fork the repository and submit pull requests with your changes.

### Licensing
This project is licensed under the MIT License. See the LICENSE file for more details.

### Contact Information
For further assistance or to report issues, please see contact information below or open an issue on the GitHub project page.

* Yaohui Wang(Cody): wangyaohui1024@outlook.com or yaohuiw2@illinois.edu
* Dr. Bo Chen: boc2@illinois.edu
* Beitong Tian: beitong2@illinois.edu
* Prof. Klara Nahrstedt: klara@illinois.edu