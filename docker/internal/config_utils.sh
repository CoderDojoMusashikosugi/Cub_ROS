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

    if declare -p container_install_scripts >/dev/null 2>&1; then
        # スペース区切りに直列化（末尾スペース除去）
        local _joined="$(printf '%s ' "${container_install_scripts[@]}")"
        export CONFIG_CONTAINER_INSTALL_SCRIPTS="${_joined% }"
    else
        export CONFIG_CONTAINER_INSTALL_SCRIPTS=""
    fi
    
    return 0
}
# Portable sed -i wrapper: use GNU sed when available, otherwise use BSD-style with an empty backup suffix.
safe_sed_inplace() {
    local expr="$1"
    local file="$2"
    # If sed supports --version it's GNU sed; otherwise assume BSD sed (macOS)
    if sed --version >/dev/null 2>&1; then
        sed -i "$expr" "$file"
    else
        sed -i '' "$expr" "$file"
    fi
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
    
    # Use sed in a portable way via safe_sed_inplace; append if key doesn't exist
    if grep -q "^IMAGE_VERSION=" "$conf_file"; then
        safe_sed_inplace "s/^IMAGE_VERSION=.*/IMAGE_VERSION=${new_version}/" "$conf_file"
    else
        echo "IMAGE_VERSION=${new_version}" >> "$conf_file"
    fi

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
        # Update existing BASE_IMAGE_VERSION using portable sed wrapper
        safe_sed_inplace "s/^BASE_IMAGE_VERSION=.*/BASE_IMAGE_VERSION=${new_version}/" "$conf_file"
    else
        # Add BASE_IMAGE_VERSION if it doesn't exist
        echo "BASE_IMAGE_VERSION=${new_version}" >> "$conf_file"
    fi

    echo "Updated BASE_IMAGE_VERSION in $conf_file to $new_version"
}