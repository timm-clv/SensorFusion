#!/bin/bash


# Has been lunched one time at the begining
# Better to not lunch it again

echo "Deploying udev rules (Olive Sensors) to the system..."

# Copy the workspace file to the system directory
sudo cp $(dirname "$0")/../rules/99-olive-networking.rules /etc/udev/rules.d/

echo "Reloading udev rules..."
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=net

echo "Installation completed successfully"
