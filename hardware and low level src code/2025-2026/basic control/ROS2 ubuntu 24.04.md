
✅ **Here is the installation procedure for ROS 2 Jazzy on a Raspberry Pi 5:**

1. Burn **Ubuntu 24.04 LTS** onto the Raspberry Pi 5 using the **Raspberry Pi Imager**.
2. Follow the installation procedure here: [ROS 2 Jazzy Installation Guide](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html).
3. When you finish the installation, type `ros2` in the terminal. If it doesn’t work, it’s probably because you still need to source your workspace.
4. Install **gedit** using this command:

   ```bash
   sudo apt update
   sudo apt install gedit -y
   ```
5. Then open `.bashrc` with:

   ```bash
   gedit ~/.bashrc
   ```

   At the end of the file, add:

   ```bash
   source /opt/ros/jazzy/setup.bash
   source ~/ros2_ws/install/setup.bash
   ```

   *(Add the `~/ros2_ws` line only if you have already created and built a workspace.)*
6. Save the file, then type `source ~/.bashrc` to reload it. Now, typing `ros2` in the terminal should work.
