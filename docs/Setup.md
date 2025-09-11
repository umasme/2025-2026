# UMASME Lunabotics - ROS 2 Development Environment

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

Follow these steps to get the development container up and running.

### Step 1: Clone the Repository into WSL

It is **crucial** that you clone this repository directly into your WSL filesystem, not onto your Windows C: drive (`/mnt/c/...`). This ensures the best performance and avoids potential permission issues.

1.  Open your WSL terminal (e.g., Ubuntu).
2.  Navigate to where you want to store your projects (e.g., your home directory).
3.  Clone the repository:
    ```bash
    git clone <your-repository-url>
    ```
4.  Navigate into the project's workspace directory (the folder containing the `.devcontainer` folder):
    ```bash
    cd <repository-name>
    ```

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
3.  **Test a GUI Application:** Run `rviz2` to confirm that graphical applications can launch.
    ```bash
    rviz2
    ```
    The Rviz2 window should appear on your Windows desktop.

---