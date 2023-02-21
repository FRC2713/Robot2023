<div align="center">
    <h1>FRC 2713 Red Hawk Robotics</h1>
    <h2>Robot Code 2023</h2>
    <img src="https://github.com/FRC2713/Robot2023/actions/workflows/gradle.yml/badge.svg" />
</div>

---

- Remove **all** 2022 FRC software.
    - Delete the folder `C:\Users\Public\wpilib`
    - Uninstall AdvantageScope
    - Uninstall NI Software
    - Uninstall NI Package Manager
    - Delete any leftover desktop shortcuts
    - You do not need to uninstall Github Desktop
- Install Github Desktop if you don't have it already
    - https://desktop.github.com/
- Install Git for Windows
    - https://git-scm.com/download/win
    - This is **separate** from Github Desktop and necessary for our version file to be generated correctly
- Install WPILib 2023
    - https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html
- Install FRC Game Tools
    - https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/frc-game-tools.html
    - When the NI Activation Wizard appears, **just close it**.
        - It's asking you for a license to activate LabVIEW and the Vision Runtime, which we don't use. You do not need a license for anything else. You can safely ignore all of the activation / license key things.
- Install PathPlanner
    - https://github.com/mjansen4857/pathplanner/releases/latest
    - I recommend just using the zip file and not installing from windows store
- Install AdvantageScope
    - https://github.com/Mechanical-Advantage/AdvantageScope/releases/latest
    - You are probably not running arm64, so download win-x64.

---

### Software static IPs
- Radio - `10.27.13.1` , set by radio config tool
- roboRIO - `10.27.13.2`, need to manually set static ip inside rio webpage config tool
  - Subnet mask `255.255.255.0`
- Driver station - `10.27.13.5`, need to manually set static ip inside windows setting
  - Subnet mask `255.0.0.0`
  - Leave gateway blank
- Limelight - `10.27.13.11`, need to manually set inside limelight webpage config
  - Gateway `10.27.13.1`
  - Subnet mask `255.255.255.0`
