#!/bin/bash
# Configuration file utility functions

# Load configuration for a specific target
load_config() {
    local target=$1
    local config_file="docker/environment/${target}.conf"
    
    if [ ! -f "$config_file" ]; then
        echo "Error: Configuration file $config_file not found"
        return 1
    fi
    
    # Load the configuration file
    source "$config_file"
    
    # Export variables for use in other scripts
    export CONFIG_BASE_IMAGE="$BASE_IMAGE"
    export CONFIG_ADDITIONAL_PKGS="$ADDITIONAL_PKGS"
    export CONFIG_IMAGE_TYPE="$IMAGE_TYPE"
    export CONFIG_IMAGE_VERSION="$IMAGE_VERSION"
    export CONFIG_BASE_IMAGE_VERSION="${BASE_IMAGE_VERSION:-$IMAGE_VERSION}"
    
    # Determine if ROS installation is needed based on base image
    if [[ "$BASE_IMAGE" == ros:* ]]; then
        export CONFIG_NEEDS_ROS_INSTALL="false"
    else
        export CONFIG_NEEDS_ROS_INSTALL="true"
    fi
    
    return 0
}

# Get the base image tag suffix
get_base_image_tag() {
    local target=$1
    load_config "$target"
    
    # Create a hash-like suffix based on base image and additional packages
    local base_hash=$(echo "${CONFIG_BASE_IMAGE}${CONFIG_ADDITIONAL_PKGS}" | md5sum | cut -c1-8)
    echo "${CONFIG_IMAGE_VERSION}_${ARCH}_${base_hash}"
}

# Get the main image tag suffix
get_main_image_tag() {
    local target=$1
    load_config "$target"
    echo "${CONFIG_IMAGE_VERSION}_${ARCH}"
}

# Get a value for a specific key from a config file
get_conf_value() {
    local conf_file=$1
    local key=$2
    
    # Check if the config file exists
    if [ ! -f "$conf_file" ]; then
        echo "Error: Config file not found: $conf_file"
        return 1
    fi
    
    # Use grep and cut to extract the value
    local value=$(grep "^${key}=" "$conf_file" | cut -d'=' -f2)
    
    # Remove leading/trailing quotes if they exist
    value=$(echo "$value" | sed -e 's/^"//' -e 's/"$//')
    
    echo "$value"
}

# Get IMAGE_VERSION from a config file
get_image_version() {
    local conf_file=$1
    get_conf_value "$conf_file" "IMAGE_VERSION"
}

# Update IMAGE_VERSION in a config file
update_image_version() {
    local conf_file=$1
    local new_version=$2
    
    # Check if the config file exists
    if [ ! -f "$conf_file" ]; then
        echo "Error: Config file not found: $conf_file"
        return 1
    fi
    
    # Use sed to update the IMAGE_VERSION
    sed -i "s/^IMAGE_VERSION=.*/IMAGE_VERSION=${new_version}/" "$conf_file"
    
    echo "Updated IMAGE_VERSION in $conf_file to $new_version"
}

# Update BASE_IMAGE_VERSION in a config file
update_base_image_version() {
    local conf_file=$1
    local new_version=$2
    
    # Check if the config file exists
    if [ ! -f "$conf_file" ]; then
        echo "Error: Config file not found: $conf_file"
        return 1
    fi
    
    # Check if BASE_IMAGE_VERSION exists
    if grep -q "^BASE_IMAGE_VERSION=" "$conf_file"; then
        # Update existing BASE_IMAGE_VERSION
        sed -i "s/^BASE_IMAGE_VERSION=.*/BASE_IMAGE_VERSION=${new_version}/" "$conf_file"
    else
        # Add BASE_IMAGE_VERSION if it doesn't exist
        echo "BASE_IMAGE_VERSION=${new_version}" >> "$conf_file"
    fi
    
    echo "Updated BASE_IMAGE_VERSION in $conf_file to $new_version"
}