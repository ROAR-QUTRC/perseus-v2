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
- (Optional) A network connection between devices.

## Running the Web-UI

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
