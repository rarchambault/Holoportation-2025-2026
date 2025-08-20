# HOLOPORTATION: Live 3D Object Capture and Transmission for CSA’s AR Astronaut Training

**By: Roxanne Archambault, Alex Turianskyj, Aleksej Dejanov, and Anh Tu Nguyen**

This capstone project is an **open-source academic implementation** of real-time 3D point cloud reconstruction and transmission, inspired by Microsoft’s concept of “Holoportation.” The current repository is based on the following repository: https://github.com/alex8ndr/Holoportation, which was updated and moved here for further updates.

## Overview

Holoportation is a system that enables real-time 3D point cloud capture, transmission, and rendering of everything within a defined zone surrounded by RGBD cameras. This project enhances astronaut training using augmented reality (AR) by transmitting real-time point cloud data to Microsoft HoloLens 2 headsets. The system processes depth and color data from multiple Orbbec Femto Bolt cameras, and transmits it in real-time, allowing trainers to include physical objects in remote training without prior modelling.

## Components
- **LiveScan3D**
    - .NET console app in C# and C++
    - Performs the real-time point cloud reconstruction from the camera data
    - Send the point cloud reconstruction in real time to connected clients (see HoloLensReceiver for more details)

- **HoloLensReceiver**
    - Unity app in C# meant for HoloLens2
    - Receives point cloud reconstructions and document detections and renders them

LiveScan3D and HoloLensReceiver are described in more detail below.

# LiveScan3D

## Overview
This project computes real time 3D reconstruction of objects using one or more Orbbec Femto Bolt RGBD camera(s) positioned around a zone called Holoport. These cameras provide both color and depth data which is then used to reconstruct any object positioned in the Holoport. The resulting 3D reconstruction is presented as a point cloud where each small point has a distinct color. Point clouds from all cameras are merged using a calibration object which, captured by each camera, allows them to transform their data from local to global space.

This project is largely based on the following open-source project: https://github.com/MarekKowalski/LiveScan3D (see the associated research paper https://ieeexplore.ieee.org/document/7335499). The main changes to this project are related to adapting it to work with Orbbec Femto Bolt cameras and optimizing it for better performance.

## Getting Started

### Hardware Prerequisites
* Windows computer running Windows 10 or 11 (older versions might be functional but have not been tested)
* At least one Orbbec Femto Bolt camera with its power cable and a USB cable connected to the computer
    * Note that the `LiveScanPlayer` application can be run without any camera connected, as it only plays point cloud recordings.
* At least one calibration marker, either printed on a sheet of paper or 3D printed using black and white filament
    * Images of the 6 different available markers can be found in this repository under `docs/calibration markers`.
    * The 3D printing files required to print the calibration marker cube can be found under `docs/calibration markers/3DPrinting`.
    * Note that `LiveScanPlayer` does not require a calibration marker.

### Software Prerequisites
All software libraries and packages required to run this project are included in the repository and will be copied to the output directory upon building the solution. Thus, there are **no software prerequisites** to run this project outside of what is contained in this repository.

## Installation and Build
Here are the few steps to follow in order to be able to run the LiveScan3D applications.

1. Clone this repository in a local directory.
2. Open the Visual Studio solution `LiveScan.sln`, located in the `LiveScan3D` folder.
3. Select the following build configuration (either using the drop-down menus at the top of the screen or under `Build > Configuration Manager`):

    | Configuration | Platform | Startup Project |
    |---------------|----------|-----------------|
    | Release       | x64      | LiveScanServer  |

4. Build the project by selecting `Build > Build Solution`.
    * Verify that a `bin` folder is created under the root directory and contains `LiveScanPlayer.exe`, `LiveScanServer.exe`, `LiveScanClient.dll` as well as several other `.dll` files.

**Note**: It is also possible to run the `LiveScanServer` or `LiveScanPlayer` application directly through Visual Studio, either in Debug or Release mode (use the `Start` button or go to `Debug > Start Debugging` (or `Start Without Debugging`)).

## Usage
Once the LiveScan3D applications have been built locally, they can be run directly from the `bin` folder where they are output during the build process.

There are two applications in the LiveScan3D project: `LiveScanServer.exe` and `LiveScanPlayer.exe`.

### LiveScanServer
The `LiveScanServer.exe` application is the application used to retrieve data from any connected cameras in real time and merge it into one colored point cloud. Thus, testing it requires having one or more Orbbec Femto Bolt cameras connected to the computer.

Here are the required steps to test this application:
1. Execute the application by double-clicking on the `LiveScanServer.exe` file (or by right-clicking on the file and selecting `Open`).
    * Verify that a UI window appears.
2. Wait for all cameras currently connected to the computer to be detected and be shown in the top left list box of the UI window as "clients".
    * Verify that the number of clients shown in the list box is the same as the number of cameras that are currently connected to the computer.
    * Verify that the serial number of each camera appears after a few seconds, in parentheses after each client index (it is initially set to `XXXXXXXXXXX`). This identifier can also be found on the cameras themselves, under a small QR code located on the label under the camera.
3. Place one or more calibration markers in the Holoport zone covered by the cameras such that each camera can see at least one of them (use the Camera application on Windows to visualize the cameras' fields of view). The default settings expect a calibration marker cube with 9 cm sides with markers 0 to 4 positioned on each visible face. With those settings, the cube will be placed at the Holoport's origin (0,0,0), with a rotation of (0,0,0) (laying flat on a table, for example).
4. If needed, adjust the settings to account for any additional markers placed in the zone by selecting `Settings` and adjusting the values under `Calibration markers`, on the top right of the Settings form.
5. Calibrate the cameras using the positioned markers by selecting `Calibrate` on the bottom left of the main UI form.
    * Verify that all connected cameras indicate "Calibrated = True" after a few seconds in the top left list box.
6. Visualize the output of the reconstruction by selecting `Show live` at the center of the main UI form.
    * Verify that a new window appears where a point cloud reconstruction is displayed and updated as objects are moved within the Holoport.

### LiveScanPlayer
The `LiveScanPlayer.exe` application is used to play recordings of point clouds that have been captured using `LiveScanServer` beforehand. A test recording in `.ply` format is provided in this repository, under `LiveScanPlayer > TestRecording`.

Here are the required steps to test this application:
1. Extract all files from the `.zip` file under `LiveScanPlayer > TestRecording` to a local directory in the same parent directory.
    * Verify that several `.ply` files are extracted.
2. Execute the application by double-clicking on the `LiveScanPlayer.exe` file (or by right-clicking on the file and selecting `Open`).
    * Verify that a UI window appears.
3. Select `Select ply files` on the left of the UI form.
    * Verify that a file explorer window appears.
4. Browse to `LiveScanPlayer > Test Recording`and find the directory where the `.ply` files were extracted earlier. Select any `.ply` file and click on `ctrl + A` to select all the `.ply` files in the folder. Click on `Open` to confirm the selection.
    * Verify that a new entry appears in the table on the right of the UI form with the selected path under `Filename` and 0 under `Current frame`.
5. Select `Start player` on the left of the UI form to start the player.
    * Verify that the number under `Current frame` increases rapidly, then goes back to 0, in a loop.
6. Select `Show live` on the bottom left of the UI form to visualize the test recording.
    * Verify that a new window appears where a point cloud reconstruction is displayed and updated rapidly. 

# HoloLens Receiver

This project is a Unity application made for HoloLens2. Its main purpose is to receive point clouds from a LiveScan3D TCP server and render them on HoloLens2.

This project is based on the following open-source project: https://github.com/MarekKowalski/LiveScan3D-Hololens, which itself was made to work with the following project: https://github.com/MarekKowalski/LiveScan3D (see the associated research paper https://ieeexplore.ieee.org/document/7335499). 

## Getting started

### Hardware Prerequisites
* HoloLens2 headset
* Unity version 2022.3.8f1 and Unity Hub (other versions may work but have not been tested)
* Windows computer running a LiveScan3D TCP server (this may also require other hardware components such as Orbbec Femto Bolt cameras, etc.

### Software Prerequisites
All software libraries and packages required to run this project are included in the Unity project. Thus, there are **no software prerequisites** to run this project outside of what is contained in this repository.

## Installation and Build
Here are the few steps to follow in order to be able to run the HoloLensReceiver application.

1. Clone this repository in a local directory.
2. Open the Unity Hub and add the Unity project located in the `HoloLensReceiver` folder by selecting `Add` and browsing to the directory where the project was cloned.
3. Select the new project that was just added (`HoloLensReceiver`) to open it.
    * Verify that the project opens after some time and that Unity does not raise any errors or messages.
4. In the Project window (on the bottom left by default), under `Assets > Scenes`, open `MainScene`.
5. In the `Hierarchy` menu of the scene (on the top left by default), select the `HoloportController` object.
6. In the `Inspector` window (on the top right by default), in the `HoloportController (Script)` component, write the IP address of the computer running the LiveScan3D TCP server under `Default Server IP Addresses`.

The project is now ready to be tested in the Unity Editor! Move to the Usage section to learn how to test the project through the Unity Editor. 

**To build and deploy the project on HoloLens2**, a few additional steps are required. Note that these steps explain how to deploy the application to HoloLens by creating an app package; other methods such as network deployment will also work.

7. In the Unity Editor, navigate to `File > Build Settings...`.
    * Verify that the `Build Settings` window opens.
8. Verify and adjust the build settings to reflect the following configuration. Note that you may need to switch the build platform before you can actually build the application.
    
    <table>
        <tr>
            <th>Platform</th>
            <td>Universal Windows Platform</td>
        </tr>
        <tr>
            <th>Architecture</th>
            <td>ARM 32-bit</td>
        </tr>
        <tr>
            <th>Build Type</th>
            <td>D3D Project</td>
        </tr>
        <tr>
            <th>Visual Studio Version</th>
            <td>Visual Studio 2022</td>
        </tr>
        <tr>
            <th>Build configuration</th>
            <td>Release</td>
        </tr>
    </table>

9. Click on `Build` to start building the project; you will need to select a folder to store the build (e.g., a folder named "Build" under the root folder).
    * Verify that the build process completes successfully after some time.
10. In the Build folder, open the Visual Studio solution file that was created (`HoloLensReceiver.sln`).
11. Select the following build configuration (either using the drop-down menus at the top of the screen or under `Build > Configuration Manager`):

    | Configuration | Platform |
    |---------------|----------|
    | Release       | ARM      |

12. Right click on the `HoloLensReceiver (Universal Windows)` project and select `Publish > Create App Packages...`.
    * Verify that the `Create App Packages` window opens.
13. In the `Select distribution method` page (the first one), click on `Next`.
14. In the `Select signing method` page, click on `Next` again.
15. In the `Select and configure packages` page, note the Output location path; this is where the app package will be output. Uncheck the `Automatically increment` checkbox under the version number. Ensure that the only Architecture / Solution Configuration combination which is checked to be created is the following:

    | Architecture | Solution Configuration |
    |--------------|------------------------|
    | ARM          | Release (ARM)          |

16. Click on `Create` to launch the app package creation.
    * Verify that the app package is created in the Output location path provided.
17. Open the HoloLens Device Portal by entering the device's IP address in a web browser.
18. Navigate to `Views > Apps` from the left menu.
19. Under `Deploy apps`, in the `Local Storage` menu, click on `Choose File` and browse to the path where the app package was created.
20. Select the app package which was just created (it should be called `HoloLensReceiver_<version>_ARM.appx`) and click on `Open`.
21. Click on `Install` to begin the installation.
    * Verify that the package is successfully installed after a few seconds (the following message should be displayed above the status bar: "Package Successfully Registered").

## Usage
As mentioned above, there are two ways to test the HoloLensReceiver application; through the Unity Editor and directly on HoloLens. Both ways require the LiveScan3D server to be running beforehand; this means that either the `LiveScanServer` or `LiveScanPlayer` application must be running on the computer which was designated as the server (both `LiveScanServer` and `LiveScanPlayer` communicate with `HoloLensReceiver` in the same way). Note that both the server computer and the device running `HoloLensReceiver` must be on the same local network. If the server is not started when the HoloLensReceiver is started, then some error messages will be displayed, and connection will be retried at regular intervals as long the HoloLensReceiver is running.

### Unity Editor
1. Ensure that the LiveScan3D server is running on the computer which was designated as the server (the IP address entered above in the `HoloportController` component).
2. Click on the Play button at the top of the Unity Editor window to launch the application.
    * Verify that the point cloud which is displayed in the LiveScan3D application is now also displayed in the `Game` window of the Unity Editor.
    * The W, A, S, D, Q, E keys can be used to move the camera around the point cloud. The R key also toggles the ability of the mouse to control the camera rotation.

### HoloLens
1. Ensure that the LiveScan3D server is running on the computer which was designated as the server (the IP address entered above in the `HoloportController` component).
2. Put on the HoloLens and launch the `HoloLensReceiver` application through the HoloLens' applications menu.
    * Verify that the point cloud which is displayed in the LiveScan3D application is now also displayed in the `Game` window of the Unity Editor (the point cloud should appear 1 meter in front of the position of your head upon launching the application).
