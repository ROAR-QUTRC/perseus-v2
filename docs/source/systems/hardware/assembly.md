# Perseus Assembly

This page contains instructions on how to assemble hardware systems on Persues. All systems have been designed so that the disassembly instructions are the same as the assembly instructions, but in reverse.

Systems in recommended order of assembly:

- Chassis
- Motors
- Suspension arms
- Mast
- Bucket

## Chassis

The chassis should never be deconstructed unless parts are being upgraded, in which case, assembly instructions will be replaced. For transport of the chassis, the Orin, networking gear, and all other components should be removed and transported in their own containers.

## Motors

1. Screw the motor into its 3d printed motor housing using flathead screws until the screw is flush with the top of the housing.
2. Attach the gearbox collet to the motor shaft by tightening the grub screw with an allen key. Ensure this is tight enough that the Allen key flexes slightly but does not bend. The use of pliers for extra leverage may be necessary to remove.
3. Then attach the motor housing to the gear box using 4 hex head screws with the collet screw hole pointing upwards.
4. Screw the motor housing lid on using 3 bolts.

:::{warning}
The motors must be put in their housing so that the following conditions are satisfied:

- The PCB on the side of the motor should be pointing to the top of the housing (The top is the curved corner that has the index label on it). This must be done so that none of the other mounting holes are covered and there is no rubbing between the motor and the cable.
- Motor housings labeled 0 and 1 should have the motors with the short cables, and 2 and 3 with the long cables.

:::

:::{note}
These should not be disassembled for travel and only removed from the suspension arms. However, if they are disassembled ensure there is tape over the open end of the gear box and collet screw hole.
:::

## Suspension arms

1. Mount the motors to the suspension arms using M5 countersink bolts and lock nuts. Ensure the motor housing labels correctly match the labels on the suspension arms, and the motor cables are the correct lengths (They should reach the pivot point on the arm).
2. Slide the wheel mounting hubs onto the motor shafts and secure them with a bolt in their center. These will be a tight fit and require wobbling back and forth. Do not use a hammer while attaching these.
3. Attach the VESC housing using hex head bolts. For correct orientation, reference the CAD models.
4. Place all the bearings into the arm mounts on the chassis, then slide the arm into the mounting hole. For the first arm, it may be easier to roll the chassis onto its side before sliding it in.
5. Secure the arm with its bolt and custom washer. It may be hard to access this with the socket wrench, so instead, secure it with the socket wrench and rotate the arm back and forth without hitting the motor housing on the chassis to ratchet the wrench.
6. Repeat for the other suspension arm.
7. Verify the threaded rods are mounted to the sway bar with the correct configuration of washers and nuts.
8. Mount the sway bar to the back of the chassis with two M18 bolts from the outside and secure with nuts and washers from the inside.
9. Screw the threaded rods onto the inside of the mounting holes on the back of the suspension arms.
10. Slide the wheels onto the wheel mounts so the tracks form an X when viewed from above. Once the wheels are on, secure them with 4 washers and nuts.

:::{note}
For travel, the sway bar and motors should be removed but not disassembled. Furthermore, it is recommended to remove the VESC housing from the suspension arms.
:::

## Mast

The mast is designed to be modular, with all fittings being mounted to the aluminium extrusions with a small bolt, washer, and t-nut. Due to this design, there are only two modules that have specific locations:

- The Unify access point mount must be at the top of the mast.
- Mount the diagonal support poles. This is typically positioned approximately 50mm below the Unify access point mount, but can be moved by replacing the support poles.

Leave a generous space at the bottom of the mast to slide into the two mounting holes at the front of the chassis.

The recommended configuration for the mast from top to bottom is:

- Unify access point mount
- Long camera mount
- Support pole mount
- Livox mount
- Depth camera mount
- short camera mount

:::{note}
For travel it is recommended that the entire mast is disassembled keeping all the t-nuts, bolts and washers screwed onto their modules.
:::

## Bucket
