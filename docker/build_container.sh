#!/bin/bash
set -eu  # Exit immediately if a command exits with a non-zero status

# Default values
BRANCH="torpedo_fin"
ROS=false
INCLUDE_EXAMPLES=false

if ! command -v docker &> /dev/null; then
    echo "❌ Docker not found. Please install Docker before running this script."
    exit 1
fi

# TODO: print warning that this image should not be shared outside of organization

echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "\033[1;33m📢 IMPORTANT SETUP INSTRUCTIONS\033[0m"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo -e "⚠️  WARNING: This image contains proprietary software and should not be shared outside of the organization."
echo -e "To clone the \033[1mHoloOcean\033[0m repository, you must:"
echo -e "  1. ✅ Follow the HoloOcean setup instructions to link your GitHub account with \033[1mEpic Games\033[0m."
echo -e "  2. 🔐 Ensure you have an \033[1mSSH key\033[0m configured for GitHub access."
echo "     (HTTPS cloning will not work.)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo ""
# Ask if user has signed EULA
read -p "Have you signed the EULA for this software? [y/n]: " eula_signed
eula_signed=$(echo "$eula_signed" | tr '[:upper:]' '[:lower:]')
if [[ "$eula_signed" != "y" ]]; then
  echo "❌ You must sign the EULA before continuing."
  exit 1
fi

read -p "Do you want to allow docker to access the display? [y/n]: " display_access
display_access=$(echo "$display_access" | tr '[:upper:]' '[:lower:]') 
if [[ "$display_access" == "y" ]]; then
  xhost +local:docker
  echo "WARNING: the command (xhost +local:docker) will need to be run again after every reboot"
fi

# Parse command-line options
while getopts "b:r" opt; do
  case ${opt} in
    b )
      BRANCH=$OPTARG
      ;;
    r )
      ROS=true
      ;;
    \? )
      echo "Usage: $0 [-b branch_name] [-r]"
      echo "  -b : Specify git branch (default: torpedo_fin)"
      echo "  -r : Copy ROS packages into container"
      exit 1
      ;;
  esac
done

if [ "$ROS" = true ]; then
  # Ask if user wants to include examples
  read -p "Do you want to include the holoocean_examples package? [y/n]: " include_examples
  include_examples=$(echo "$include_examples" | tr '[:upper:]' '[:lower:]')
  if [[ "$include_examples" == "y" ]]; then
    INCLUDE_EXAMPLES=true
  fi
fi

# Function to create tmp dir and copy necessary files
create_copy_tmp() {
    echo "🔧 Setting up temporary directory..."
    mkdir -p tmp/ros_packages

    echo "📥 Cloning HoloOcean repository..."
    git clone git@github.com:byu-holoocean/HoloOcean.git tmp/HoloOcean
    cd tmp/HoloOcean

    echo "📂 Checking out branch: $BRANCH"
    git checkout "$BRANCH"

    # Remove engine files
    rm -rf engine
    cd ../..

    if [ "$ROS" = true ]; then
      echo "📄 Copying ROS packages..."
      cp -r ../holoocean_main tmp/ros_packages/
      cp -r ../holoocean_interfaces tmp/ros_packages/

      if [ "$INCLUDE_EXAMPLES" = true ]; then
          echo "📄 Including examples..."
          cp -r ../holoocean_examples tmp/ros_packages/
      fi
    fi
}

# Check for existing tmp directory
if [ -d "tmp" ]; then
  echo "📁 'tmp/' directory already exists."
  read -p "Do you want to skip cloning and copying ROS packages? [y/n]: " skip_tmp
  skip_tmp=$(echo "$skip_tmp" | tr '[:upper:]' '[:lower:]')
  if [[ "$skip_tmp" == "y" ]]; then
    echo "⚠️  Skipping setup. Will only build the Docker image with existing tmp directory."
  else
    echo "🧹 Removing old tmp directory..."
    rm -rf tmp/
    create_copy_tmp
  fi
else
  create_copy_tmp
fi

echo "🐳 Building Docker image..."
docker compose build

# TODO Ask if user wants to clean up
echo "🧹 Cleaning up temporary files..."
rm -rf tmp/

# Optional: Ask if user wants to clean up
read -p "Would you like to start the container in the background? [y/n]: " startup
startup=$(echo "$startup" | tr '[:upper:]' '[:lower:]')
if [[ "$startup" == "y" ]]; then
  echo "🐳 Starting up container"
  docker compose up -d
fi

echo "✅ Done!"
