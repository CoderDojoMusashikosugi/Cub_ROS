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
    export CONFIG_IMAGE_NAME="$IMAGE_NAME"
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

# Get the unified image name for similar configurations
get_unified_image_name() {
    local target=$1
    load_config "$target"
    echo "$CONFIG_IMAGE_NAME"
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

# Check if two targets share the same image configuration
targets_share_image() {
    local target1=$1
    local target2=$2
    
    load_config "$target1"
    local image1="$CONFIG_IMAGE_NAME"
    local base1="$CONFIG_BASE_IMAGE"
    local pkgs1="$CONFIG_ADDITIONAL_PKGS"
    
    load_config "$target2"
    local image2="$CONFIG_IMAGE_NAME"
    local base2="$CONFIG_BASE_IMAGE"
    local pkgs2="$CONFIG_ADDITIONAL_PKGS"
    
    if [ "$image1" = "$image2" ] && [ "$base1" = "$base2" ] && [ "$pkgs1" = "$pkgs2" ]; then
        return 0
    else
        return 1
    fi
}

# Update IMAGE_VERSION for all config files that share the same IMAGE_NAME
update_shared_image_versions() {
    local target_config=$1
    local new_version=$2
    
    # Load the current target's config to get IMAGE_NAME
    load_config "$target_config"
    local target_image_name="$CONFIG_IMAGE_NAME"
    
    echo "Updating IMAGE_VERSION to $new_version for all configs with IMAGE_NAME=$target_image_name"
    
    # Find all config files in the environment directory
    for config_file in docker/environment/*.conf; do
        if [ -f "$config_file" ]; then
            # Extract the config name from the file path
            local config_name=$(basename "$config_file" .conf)
            
            # Load this config to check its IMAGE_NAME
            load_config "$config_name"
            
            # If IMAGE_NAME matches, update the IMAGE_VERSION
            if [ "$CONFIG_IMAGE_NAME" = "$target_image_name" ]; then
                echo "  Updating $config_file"
                sed -i "s/^IMAGE_VERSION=.*/IMAGE_VERSION=${new_version}/" "$config_file"
            fi
        fi
    done
}

# Update BASE_IMAGE_VERSION for all config files that share the same IMAGE_NAME and BASE_IMAGE
update_shared_base_versions() {
    local target_config=$1
    local new_version=$2
    
    # Load the current target's config to get IMAGE_NAME and BASE_IMAGE
    load_config "$target_config"
    local target_image_name="$CONFIG_IMAGE_NAME"
    local target_base_image="$CONFIG_BASE_IMAGE"
    
    echo "Updating BASE_IMAGE_VERSION to $new_version for all configs with IMAGE_NAME=$target_image_name and BASE_IMAGE=$target_base_image"
    
    # Find all config files in the environment directory
    for config_file in docker/environment/*.conf; do
        if [ -f "$config_file" ]; then
            # Extract the config name from the file path
            local config_name=$(basename "$config_file" .conf)
            
            # Load this config to check its IMAGE_NAME and BASE_IMAGE
            load_config "$config_name"
            
            # If IMAGE_NAME and BASE_IMAGE match, update the BASE_IMAGE_VERSION
            if [ "$CONFIG_IMAGE_NAME" = "$target_image_name" ] && [ "$CONFIG_BASE_IMAGE" = "$target_base_image" ]; then
                echo "  Updating $config_file"
                # Check if BASE_IMAGE_VERSION exists in the config file
                if grep -q "^BASE_IMAGE_VERSION=" "$config_file"; then
                    # Update existing BASE_IMAGE_VERSION
                    sed -i "s/^BASE_IMAGE_VERSION=.*/BASE_IMAGE_VERSION=${new_version}/" "$config_file"
                else
                    # Add BASE_IMAGE_VERSION if it doesn't exist
                    echo "BASE_IMAGE_VERSION=${new_version}" >> "$config_file"
                fi
            fi
        fi
    done
}
