# Web UI

External libraries will not be documented here: The UI library used is [shadcn-svelte](https://next.shadcn-svelte.com), with [roslibjs](https://robotwebtools.github.io/roslibjs/index.html) used for communication with the ROS stack.

:::{note}
All file paths in this page are relative to the root of the Web UI project (`perseus-v2/software/web-ui`)
:::

## Server

SvelteKit provides a number of methods of assisting with server communication, however I would recommend using the node server and WebSockets to prevent weird bugs from components re-rendering. Since Svelte doesn't provide a native interface for creating custom node servers an injected server is used. For development and testing use write your node server code in the vite plugin inside `vite.config.ts` and copy this code into `web-ui/src/server/server.js` for production (alternatively create a new file that exports a function and call it in both then develop from that file).

## Widgets

To begin developing a widget run the command: `./create-widget.sh <file-name>`. The `file-name` argument is just the name of the file that contains the widget and **NOT** the widget name. This should generate a new file `/src/lib/widgets<file-name>.svelte` with this template contents:

```svelte
<script lang="ts" module>
	import type { WidgetSettingsType } from '$lib/scripts/state.svelte';

	export const name = 'New Widget';

	export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
		groups: {}
	});
</script>

<script lang="ts">
	// Widget logic goes here
</script>

<p>New component</p>
```

- The first script tag with the `module` property is a server only module. This is used here as it is only run once when the component is first loaded and it also allows for exports that are used to expose some properties. You likely will not need to write your own code here.

  - **name -** This string is the unique name of the widget that will be displayed at the top of the widget and is used to ensure duplicates of widgets are not loaded.
  - **settings -** This is the object that structures the settings/state panel of each widget, this will automatically update across all connected devices when saved and will persist between sessions. Here is an example settings object and how to access the value of each setting:

    ```ts
    export const settings: WidgetSettingsType = $state<WidgetSettingsType>({
      groups: {
        general: {
          textSetting: {
            type: "text", // options are: text, number, switch, select, button
            value: "Some text", // This is a default value
          },
          hereIsAButton: {
            type: "button",
            action: () => {
              // action is executed on button press
              console.log("Button clicked");
              return "Success"; // string return value is printed in a toast
            },
          },
        },
        Advanced: {
          booleanSwitch: {
            type: "switch",
            description: "This is a switch", // fields with descriptions get a question mark next to their label
          },
          selectOptions: {
            type: "select",
            options: [
              { value: "1", label: "Option 1" },
              { value: "2", label: "Option 2" },
              { value: "3", label: "Option 3" },
            ],
          },
        },
      },
    });

    console.log(settings.groups.Advanced.booleanSwitch.value); // logs the state of the switch
    ```

- The second script tag is where your code should go as it is run client side when the component is rendered. The settings can be accessed here without an import and to listen for changes wrap the value you want to check in either a `$derived` or `$effect` rune. Furthermore, use `toast('message')` to send a toast from your widget.
- The remainder of the file is where mark down goes. Tailwind classes can be used on widgets for styling or a `<style></style>` block, although the former is strongly preferred if possible.

### Widget manager internals

Internally the widgets are made up of several components:

- The custom widget components (found in: `/src/lib/widgets/`)
- Widget state script (`/src/lib/scripts/state.svelte.ts`)
- Widget canvas (`/src/lib/components/widget-canvas.svelte`)
- Widget wrapper (`/src/lib/components/widget.svelte`)

**Custom Widget Components**
All the files in the top level of the `/src/lib/widgets/` directory are loaded as widgets by the server script in `+layout.svelte` before being converted to fit the widget type defined as:

```ts
export interface WidgetType {
  name: string;
  description?: string;
  group?: WidgetGroupType;
  isRosDependent?: boolean;
  component: Component;
  settings: WidgetSettingsType;
  layoutProps?: {
    x: number;
    y: number;
    w: number;
    h: number;
  };
}

// represents the settings of the widget
export interface WidgetSettingsType {
  groups: Record<
    string,
    Record<
      string,
      {
        type: "text" | "number" | "select" | "switch" | "button" | "readonly";
        description?: string;
        value?: string;
        options?: { value: string; label: string }[]; // these are not saved as options are typically session dependent
        disabled?: boolean;
        action?: () => string | null;
      }
    >
  >;
}
```

**Widget state**
This file holds the reactive state of the layouts and widgets. The main function it exports is `getWidgetsByLayoutId` which is used to get a list of the widgets currently in use. Additionally, since the widget's persistent state is stored in the layout database, it uses the loaded widgets as a template and populates them using the saved state data.

**Widget canvas**
The widget canvas is responsible for the drag and drop functionality of the Web UI. The drag and drop engine is [svelte-grid-extended](https://github.com/cuire/svelte-grid-extended) which uses svelte 4.2. While this library works, future releases of Svelte might drop support for this in which case it will need to be ported, rewritten, or replaced.
Each widget has its location saved to the database on every movement.

**Widget wrapper**
This component does two things, create an instance of the custom widget component and generate the widget settings menu from the `WidgetSettingsType` instance.

## WebRTC Videos

### Camera server

The function of the camera server is to:

- Provide an interface to connected linux video devices
- Generate Gstreamer pipelines and manage instances
- Communicate with the main web server

**Linux device interface:**
In the folder: `/dev/v4l/by-id/` each camera has its ID listed. A node file watcher is used to get a live list of devices in this folder allowing for connect and disconnect handling.
:::{note}
Each video device has 2 files associated with it. The one with the suffix `-index0` is the correct one and all other only contain meta data about the camera.
:::

**Gstreamer pipelines**
Each Gstreamer instance is tracked with the videos resolution and transform so the server known when a restart is necessary. Each pipeline uses these plugins: `webrtcsink`, `v4l2src`, `videoflip`, `videoconvert` to apply all the required processing as seen in this example pipeline:

```
gst-launch1.0 webrtcsink stun-server=NULL name=ws signaller::uri="ws://{web server host}:8443" meta="meta,device={device name}" v4l2src device=/dev/v4l/by-id/{device name} ! video/x-raw,width=320,height=240 ! videoconvert ! videoconvert method=none ! ws.
```

:::{note}
The `signaller::uri="ws://hostname:8443"` property sets the signalling server to location as it is run by the main web server. Furthermore, the `meta` property assigns meta data to each WebRTC stream allowing the stream origin to be determined.
:::

**Web server communication**
The communication with the web server is conducted via a WebSocket with the goal of sending information about the available devices to the client and then initialising requested streams. All messages are sent using the following interface.

```ts
interface CameraEventType {
  type: "camera";
  action:
    | "group-description"
    | "kill"
    | "request-groups"
    | "request-stream"
    | "group-terminated"
    | "device-disconnect";
  data: {
    devices?: string[];
    resolution?: { width: number; height: number };
    transform?: videoTransformType;
    forceRestart?: boolean;
  };
}
```

## ROS2 Interface

## CAN Bus Interface
