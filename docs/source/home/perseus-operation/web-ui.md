# Web-UI

The guide explains how to access the Web-UI and then connect to rosbridge and video from cameras.

## Prerequisites

To begin this guide the following conditions must be met:

- Access to a linux device with the perseus-v2 repo cloned and up to date:

  ```console
  git checkout main
  git pull
  ```

- Access to at least one camera (This includes built in webcams for laptops).
- Perseus fully powered on with compute running <project:/home/perseus-operation/power-on.md>
- A network connection to connect to Perseus remotely.

## Running the Web-UI
:::{info}
All of the command for running the web-ui should be run from inside the web-ui folder: `perseus-v2/software/web-ui`
:::

```shell
yarn # This installs dependencies
yarn build # Build the server
yarn start # Runs the built server on port 3000
```

**Note:** Any hydration related errors can be ignored

## Running Cameras

All devices with a cameras connected must have an instance of the camera server running on them. This can be run with:

```shell
yarn camera
```

:::{tip}
If this is the firts time the camera server has been run on this device the dependency `tsx` will need to be installed. This required an internet connection and can be done with command: `yarn camera-online`. Using the `-online` variant does require internet access.
:::

## Connecting to ROS2

To be able to access ROS2 data in the web-ui the ros web bridge must be running (this is included in the main Perseus bringup). Once this is running then connections are managed through the connection panel located at the top middle of the screen. If it has not automatically connected (indicated with a green circle and the word connected) then this means that ip is incorrect. To change the ip, select the middle button in the connection menu and if perseus bring up is running on the same machine as the web-ui then choose 'same as host' otherwise you will need to input either the device hostname or ip address that is running the main perseus bring up.

To disconnect from the ros bridge simply press the 'connected' button.