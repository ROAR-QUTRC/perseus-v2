#!/bin/bash

if [ "$EUID" -eq 0 ]; then
  echo "Error: Please run this script as a non-root user"
  exit 1
fi

if [ -z "$1" ]; then
  echo "Error: Please provide a component name"
  echo "Usage: ./create-component.sh ComponentName"
  exit 1
fi

WIDGET_NAME="$(echo "$1" | sed -r 's/(^|-)([a-z])/\U\2/g' | sed 's/^./\l&/')"
WIDGET_DIR="./src/lib/widgets"

# Create widget directory if it doesn't exist
mkdir -p "$WIDGET_DIR"

WIDGET_PATH="$WIDGET_DIR/$WIDGET_NAME.svelte"

# Check if file already exists
if [ -f "$WIDGET_PATH" ]; then
  echo "Error: Widget $WIDGET_NAME already exists at $WIDGET_PATH"
  exit 1
fi

cat >"$WIDGET_PATH" <<EOF
<script lang="ts" module>
	// This is to expose the widget settings to the panel. Code in here will only run once when the widget is first loaded.
	import type { WidgetSettingsType } from '\$lib/scripts/state.svelte';

	export const name = 'New Widget';
	// These properties are optional
	// export const description = 'Description of the widget goes here';
	// export const group: WidgetGroupType = 'Group Name';
	// export const isRosDependent = true; // Set to true if the widget requires a ROS connection

	export const settings: WidgetSettingsType = \$state<WidgetSettingsType>({
		groups: {}
	});
</script>

<script lang="ts">
	// import { getRosConnection } from '\$lib/scripts/ros-bridge.svelte'; // ROSLIBJS docs here: https://robotwebtools.github.io/roslibjs/Service.html
	// import ROSLIB from 'roslib';

	// Widget logic goes here
</script>

<p>New component</p>
EOF

echo "Created component $WIDGET_NAME at $WIDGET_PATH"
