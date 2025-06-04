#!/bin/bash
# TODO: remove ROS packages
set -e  # Exit immediately if a command exits with a non-zero status

# Default values
BRANCH="torpedo_fin"
INCLUDE_EXAMPLES=false

# TODO: print warning that this image should not be shared outside of organization
echo "⚠️  WARNING: This image contains proprietary software and should not be shared outside of the organization."

# Ask if user has signed EULA
echo "To clone the HoloOcean Repository please follow the HoloOcean instructions to setup your github account with Epic Games"
read -p "Have you signed the EULA for this software? [y/n]: " eula_signed
eula_signed=$(echo "$eula_signed" | tr '[:upper:]' '[:lower:]')
if [[ "$eula_signed" != "y" ]]; then
  echo "❌ You must sign the EULA before continuing."
  exit 1
fi

# Ask if user wants to include examples
read -p "Do you want to include the holoocean_examples package? [y/n]: " include_examples
include_examples=$(echo "$include_examples" | tr '[:upper:]' '[:lower:]')
if [[ "$include_examples" == "y" ]]; then
  INCLUDE_EXAMPLES=true
fi

read -p "Do you want to allow docker to access the display? [y/n]: " display_access
display_access=$(echo "$display_access" | tr '[:upper:]' '[:lower:]') 
if [[ "$display_access" == "y" ]]; then
  xhost +local:docker
  echo "WARNING: the command (xhost +local:docker) will need to be run again after every reboot"
fi

# Parse command-line options
while getopts "b:" opt; do
  case ${opt} in
    b )
      BRANCH=$OPTARG
      ;;
    \? )
      echo "Usage: $0 [-b branch_name]"
      echo "  -b : Specify git branch (default: torpedo_fin)"
      exit 1
      ;;
  esac
done

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

    echo "📄 Copying ROS packages..."
    cp -r ../holoocean_main tmp/ros_packages/
    cp -r ../holoocean_interfaces tmp/ros_packages/

    if [ "$INCLUDE_EXAMPLES" = true ]; then
        echo "📄 Including examples..."
        cp -r ../holoocean_examples tmp/ros_packages/
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

# Ask if user wants to clean up
echo "🧹 Cleaning up temporary files..."
rm -rf tmp/

# Optional: Ask if user wants to clean up
read -p "Would you like to start the conatiner in the background? [y/n]: " startup
startup=$(echo "$startup" | tr '[:upper:]' '[:lower:]')
if [[ "$startup" == "y" ]]; then
  echo "🐳 Starting up contianer"
  docker compose up -d
fi

echo "✅ Done!"
