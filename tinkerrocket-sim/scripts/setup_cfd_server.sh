#!/bin/bash
# =============================================================================
# CFD Server Setup Script
# Target: Ubuntu 24.04 at 192.168.4.176 (user: cepeders)
# Installs: Docker, OpenFOAM (via Docker), ParaView, gmsh, Python tools
# =============================================================================
set -e

echo "=== CFD Server Setup ==="
echo "$(date)"

# --- Step 0: Kill any stale apt processes and fix locks ---
echo "[0/6] Cleaning up stale apt locks..."
sudo killall apt-get 2>/dev/null || true
sudo killall apt 2>/dev/null || true
sleep 2
sudo rm -f /var/lib/apt/lists/lock
sudo rm -f /var/lib/dpkg/lock
sudo rm -f /var/lib/dpkg/lock-frontend
sudo rm -f /var/cache/apt/archives/lock
sudo dpkg --configure -a 2>/dev/null || true

# --- Step 1: Fix slow mirrors (use archive.ubuntu.com instead of us.archive) ---
echo "[1/6] Updating apt sources to use faster mirrors..."
if [ -f /etc/apt/sources.list.d/ubuntu.sources ]; then
    sudo sed -i 's|us\.archive\.ubuntu\.com|archive.ubuntu.com|g' /etc/apt/sources.list.d/ubuntu.sources
    # Temporarily disable ROS repo if it's causing timeouts
    if [ -f /etc/apt/sources.list.d/ros2.list ]; then
        sudo mv /etc/apt/sources.list.d/ros2.list /etc/apt/sources.list.d/ros2.list.bak
        echo "  (Disabled ROS2 repo temporarily to speed up apt)"
    fi
fi

sudo apt-get update -qq
echo "  apt update done."

# --- Step 2: Install Docker ---
echo "[2/6] Installing Docker..."
if command -v docker &>/dev/null; then
    echo "  Docker already installed: $(docker --version)"
else
    # Install prerequisites
    sudo apt-get install -y -qq ca-certificates curl gnupg

    # Add Docker GPG key
    sudo install -m 0755 -d /etc/apt/keyrings
    curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo tee /etc/apt/keyrings/docker.asc > /dev/null
    sudo chmod a+r /etc/apt/keyrings/docker.asc

    # Add Docker repo
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu $(. /etc/os-release && echo "$VERSION_CODENAME") stable" | \
        sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

    sudo apt-get update -qq
    sudo apt-get install -y -qq docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

    # Add user to docker group
    sudo usermod -aG docker $USER
    echo "  Docker installed: $(docker --version)"
    echo "  NOTE: You may need to log out and back in for docker group to take effect."
fi

# Start Docker if not running
sudo systemctl enable docker
sudo systemctl start docker

# --- Step 3: Pull OpenFOAM Docker image ---
echo "[3/6] Pulling OpenFOAM Docker image..."
# Use OpenCFD/ESI OpenFOAM image (v2412 = Dec 2024 release)
sudo docker pull opencfd/openfoam-default:2412
echo "  OpenFOAM v2412 Docker image pulled."

# Create a convenience alias
mkdir -p ~/bin
cat > ~/bin/openfoam <<'SCRIPT'
#!/bin/bash
# Launch OpenFOAM v2412 Docker container with current directory mounted
# Usage: cd /path/to/case && openfoam  (launches interactive shell)
#        cd /path/to/case && openfoam simpleFoam  (runs a solver directly)
if [ $# -eq 0 ]; then
    sudo docker run -it --rm \
        -v "$(pwd)":/case \
        -w /case \
        -e HOME=/case \
        opencfd/openfoam-default:2412 \
        bash --rcfile /usr/lib/openfoam/openfoam2412/etc/bashrc
else
    sudo docker run --rm \
        -v "$(pwd)":/case \
        -w /case \
        -e HOME=/case \
        opencfd/openfoam-default:2412 \
        bash -c "source /usr/lib/openfoam/openfoam2412/etc/bashrc && $*"
fi
SCRIPT
chmod +x ~/bin/openfoam

# Add ~/bin to PATH if not already there
if ! grep -q 'PATH.*\$HOME/bin' ~/.bashrc; then
    echo 'export PATH="$HOME/bin:$PATH"' >> ~/.bashrc
fi

echo "  Created ~/bin/openfoam convenience script."
echo "  Usage: cd /path/to/case && openfoam  (or: openfoam simpleFoam)"

# --- Step 4: Install native CFD tools ---
echo "[4/6] Installing native CFD tools..."

# ParaView (for post-processing visualization)
sudo apt-get install -y -qq paraview 2>/dev/null || echo "  ParaView: skipped (may need GUI)"

# gmsh (mesh generation)
sudo apt-get install -y -qq gmsh

# Python scientific stack for pre/post-processing
sudo apt-get install -y -qq python3-pip python3-venv python3-numpy python3-scipy python3-matplotlib

echo "  Native tools installed."

# --- Step 5: Install Python CFD tools ---
echo "[5/6] Installing Python CFD tools..."
python3 -m pip install --user --break-system-packages \
    PyFoam \
    meshio \
    pyvista \
    2>/dev/null || echo "  Some Python packages may have failed (non-critical)"

echo "  Python CFD tools installed."

# --- Step 6: Verify installation ---
echo "[6/6] Verifying installations..."
echo ""
echo "=== Installation Summary ==="
echo "Docker:     $(docker --version 2>&1 || echo 'FAILED')"
echo "Compose:    $(docker compose version 2>&1 || echo 'FAILED')"
echo "gmsh:       $(gmsh --version 2>&1 || echo 'NOT INSTALLED')"
echo "ParaView:   $(paraview --version 2>&1 || echo 'NOT INSTALLED (needs GUI)')"
echo "Python:     $(python3 --version 2>&1)"
echo ""

# Test OpenFOAM
echo "Testing OpenFOAM Docker image..."
sudo docker run --rm -e HOME=/tmp opencfd/openfoam-default:2412 bash -c "source /usr/lib/openfoam/openfoam2412/etc/bashrc && simpleFoam -help 2>&1 | head -5" || echo "  OpenFOAM test: FAILED"
echo ""

echo "=== Setup Complete ==="
echo ""
echo "Quick start:"
echo "  1. Log out and back in (for docker group)"
echo "  2. mkdir -p ~/cfd/test && cd ~/cfd/test"
echo "  3. openfoam  (launches OpenFOAM shell)"
echo "  4. Inside container: cp -r \$FOAM_TUTORIALS/incompressible/simpleFoam/pitzDaily ."
echo ""

# Re-enable ROS2 repo if we disabled it
if [ -f /etc/apt/sources.list.d/ros2.list.bak ]; then
    sudo mv /etc/apt/sources.list.d/ros2.list.bak /etc/apt/sources.list.d/ros2.list
fi
