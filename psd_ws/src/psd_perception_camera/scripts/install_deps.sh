#!/bin/bash

echo "Installing Python dependencies for cone_detector..."

# Update pip
pip3 install --upgrade pip

# Install Python dependencies
pip3 install -r requirements.txt

# Install TensorRT (if not already installed)
# Note: TensorRT installation depends on your system configuration
# Follow NVIDIA's official installation guide for your platform

echo "Dependencies installation complete!"
echo "Make sure TensorRT is properly installed for your system."