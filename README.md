# NeutronDrive
Java library for Swerve Drive without all the complexity

## Required Vendor Libraries

NeutronDrive depends on the following vendor libraries. They must be installed in your robot project as vendordeps before NeutronDrive will work.

| Library | Version | Install JSON |
|---------|---------|--------------|
| WPILib | 2026.2.1 | Included with WPILib installation |
| CTRE Phoenix 6 | 26.1.1 | https://maven.ctr-electronics.com/release/com/ctre/phoenix6/latest/Phoenix6-frc2026-latest.json |
| REVLib | 2026.0.4 | https://software-metadata.revrobotics.com/REVLib-2026.json |

### Installing Vendordeps in VS Code

1. Open the WPILib Command Palette (`Ctrl+Shift+P`)
2. Run **WPILib: Manage Vendor Libraries**
3. Select **Install new libraries (online)**
4. Paste the JSON URL for each library above
