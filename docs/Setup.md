# University of Miami　ASME Lunabotics - ROS 2 Development Environment

This guide provides step-by-step instructions for setting up and using the ROS 2 development environment for this project. This setup uses a Docker container managed by Visual Studio Code to ensure that every developer has an identical and consistent environment, eliminating "it works on my machine" issues.

---

## Prerequisites

Before you begin, ensure you have the following software installed and configured on your host machine. **Windows 11 is highly recommended**.

* **Git:** For version control.
* **WSL 2:** The Windows Subsystem for Linux. An [Ubuntu 22.04](https://ubuntu.com/wsl) distribution is recommended.
* **Docker Desktop:** Must be configured to use the **WSL 2 based engine**. You can verify this in Docker's settings: `Settings > General > Use the WSL 2 based engine`. 
* **Visual Studio Code:** The code editor.
* **VS Code Dev Containers Extension:** Search for `ms-vscode-remote.remote-containers` in the VS Code extensions panel and install it.

---

## Getting Started

Follow these steps to get the development container up and running.

### Step 1: Clone the Repository into Linux

1.  Open your WSL terminal (e.g., Ubuntu).
2.  Navigate to where you want to store your projects (e.g., your home directory).
3.  Clone the repository:

    ```bash
    git clone https://github.com/umasme/2025-2026
    ```
4.  Navigate into the project's workspace directory (the folder containing the `.devcontainer` folder).

### Step 2: Open the Project in VS Code

Launch VS Code directly from your WSL terminal within the project folder.

```bash
code .
```

### Step 3: Reopen in Container

VS Code will detect the `.devcontainer` configuration folder and display a notification in the bottom-right corner.

* Click the **"Reopen in Container"** button.

The first time you do this, VS Code will build the Docker image. This may take several minutes, as it needs to download the ROS 2 base image and install all the necessary tools. Subsequent launches will be much faster.

### Step 4: Verify Your Environment

Once the container is built and running, you can verify that everything is working correctly.

1.  **Check the Connection:** Look at the bottom-left corner of the VS Code window. It should show a green indicator with the text `Dev Container: UMASME ROS 2 Dev Container`. 
2.  **Open a Terminal:** Open a new terminal in VS Code (**Ctrl+`**). This terminal is running *inside* the container.
3.  **Add the ROS2 setup:** While in the terminal run the following commands.
     ```bash
    source /opt/ros/humble/setup.bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "source ~/ws/install/setup.bash" >> ~/.bashrc
    ```
4.  **Test a GUI Application:** Run `rviz2` to confirm that graphical applications can launch.
    ```bash
    rviz2
    ```
    The Rviz2 window should appear on your Windows desktop.

---

## Development Workflow

You are now ready to start developing!

* **Building the Workspace:** To compile your ROS 2 packages, run `colcon` from the workspace root (`/home/devuser/ws/`):
    ```bash
    colcon build --symlink-install
    ```
* **Sourcing Your Workspace:** After a successful build, remember to source your workspace to make your packages available to ROS 2. This is already added to your `.bashrc`, so it will be sourced automatically in any **new** terminal.
* **Running Your Nodes:** Use `ros2 run` to launch nodes from your packages.
    ```bash
    ros2 run your_package_name your_executable_name
    ```
* **Committing Code:** Use the **Source Control panel** in the VS Code Activity Bar on the left to stage, commit, and push your changes. This is the direct replacement for using an external tool like GitHub Desktop.

---

## Troubleshooting

### Build Fails or Runs Very Slowly

* **Problem:** The container build fails with a generic error, or the process is extremely slow.
* **Cause:** This almost always happens because the project folder is located on the Windows filesystem (e.g., `C:\Users\...`) instead of the WSL filesystem.
* **Solution:** You must move your entire project folder into WSL.
    1.  Close VS Code.
    2.  Open a WSL terminal.
    3.  Use the `mv` (move) command to relocate your project. For example, if your project is in your Windows `Documents` folder, you would run:
        ```bash
        # This command moves a folder from your Windows Documents to a 'projects' folder in WSL
        # Make sure to replace 'YourUser' and 'YourProjectFolder' with your actual folder names
        mv /mnt/c/Users/YourUser/Documents/YourProjectFolder ~/projects/
        ```
    4.  Navigate into the new project location inside WSL and relaunch VS Code with `code .`.
