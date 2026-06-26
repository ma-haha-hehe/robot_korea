# udev Rules for O3P Cameras

## Why This Is Needed

On Linux, USB devices are owned by `root` by default. Without a udev rule in
place, applications that use the O3P API must be run as root (or with `sudo`)
in order to open the camera. The rule in `11-o3p.rules` instructs the kernel's
device manager (udev) to:

- Set the permission mode to `0666` (world-readable/writable) for recognised
  O3P USB device IDs, so that any user can access the device.
- Assign ownership to the `plugdev` group, allowing group-based access control.
- Create a stable `/dev/o3p-*` symlink each time a camera is plugged in.

## Installation

1. **Copy the rules file** to the udev rules directory:

   ```bash
   sudo cp 11-o3p.rules /etc/udev/rules.d/
   ```

2. **Reload the udev rules** so the change takes effect without rebooting:

   ```bash
   sudo udevadm control --reload-rules
   sudo udevadm trigger
   ```

3. **Add your user to the `plugdev` group** (if not already a member):

   ```bash
   sudo usermod -aG plugdev $USER
   ```

   Log out and back in (or run `newgrp plugdev` in the current shell) for the
   group membership to take effect.

4. **Reconnect the camera.** Unplug and re-plug the USB cable so udev
   processes the new rule for the device.

## Verifying the Installation

After reconnecting the camera, confirm that the device node has the expected
permissions:

```bash
ls -l /dev/o3p-*
```

You should see permissions similar to `crw-rw-rw-` and the group set to
`plugdev`.

To check that your user belongs to `plugdev`:

```bash
groups $USER
```

## Uninstallation

```bash
sudo rm /etc/udev/rules.d/11-o3p.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
```
