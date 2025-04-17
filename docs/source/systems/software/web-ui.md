# Perseus-UI

The Perseus-UI is a web UI that is almost entirely independent of the rover core software. The UI is a SvelteKit + Vite app with an injected Node.js server. There are two main components to Perseus-UI:

- Widgets -> Typically a single svelte component that provide an interface for a specific functionality on the rover.
- Layouts -> A group of widgets that are regularly used together.

## Running Perseus-UI

The Rover UI uses SvelteKit meaning that it does not need a server to be run and can be started by running these commands in the root directory:

```shell
yarn # This installs dependencies
yarn build # Build the server
yarn start # Runs the built server on port 3000
```

**Note:** Any hydration related errors can be ignored

:::{note}
If you don't have node/yarn I'd recommend these steps:

Install node version manager with this script (you will need to restart your shell once its installed):

```
curl -o- https://raw.githubusercontent.com/nvm-sh/nvm/v0.40.1/install.sh | bash
```

Install the latest lts version of node:

```
nvm install --lts
```

If you have other versions of node already installed check the [nvm docs](https://github.com/nvm-sh/nvm) on how to setup environments or change the active node version.

Finally, install yarn with:

```
npm install --global yarn
```

:::

## Developing for the UI

### Client

The UI library used is [shadcn-svelte](https://next.shadcn-svelte.com), with [roslibjs](https://robotwebtools.github.io/roslibjs/index.html) used for communication with the ROS stack.

### Server

SvelteKit provides a number of methods of assisting with server communication, however I would recommend using the node server and WebSockets to prevent weird bugs from components re-rendering. Since Svelte doesn't provide a native interface for creating custom node servers an injected server is used. For development and testing use write your node server code in the vite plugin inside `vite.config.ts` and copy this code into `web-ui/src/server/server.js` for production (alternatively create a new file that exports a function and call it in both then develop from that file).

### Widgets

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
