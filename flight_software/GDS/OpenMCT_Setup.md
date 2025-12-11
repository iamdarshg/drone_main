# OpenMCT Setup Guide

This guide provides instructions on how to set up OpenMCT to connect to the F' Ground Data System (GDS) and display the vehicle's telemetry.

## Prerequisites

*   [OpenMCT](https://openmct.org/) installed and running.
*   The F' GDS is running and connected to the flight software.

## Configuration

1.  **Add a new Telemetry Server.**
    *   In OpenMCT, click on the "Create" button and select "Telemetry Server".
    *   Enter the following information:
        *   **Name:** F' GDS
        *   **URL:** `http://localhost:50000`
    *   Click "Save".

2.  **Create a new Display Layout.**
    *   Click on the "Create" button and select "Display Layout".
    *   Give the layout a name (e.g., "Flight Telemetry").
    *   Add the telemetry channels to the layout by dragging them from the telemetry tree on the left.

3.  **View the Telemetry.**
    *   Click on the "View" button to see the telemetry data.

This guide will be updated as the GDS is further developed.
