# FRC 2024 Crecendo - #8608 Alpha bots - PizzaJoe 
This is our 2nd cadded and java programmed Bot (3 year team). statbotics.io places it as one of the top 100 bots in the world.
our first event we were captains of the 3rd alliance, in our second event we WON as the 3rd alliance, at michigan state we were captains of the 4th alliance, and at worlds we were captains of the 6th alliance. 
## Cad
The Cad for the Robot this code goes to is here https://grabcad.com/library/frc-8608-alpha-bots-worlds-bot-1

## Autonomous

The Auton is a 4 (almost 5 piece). Since the whole the robot is made in the command based structure from WPI-lib; all commands used by the driver are used by the auton. therefore our robot and auton were ready together for week 1. 

we had outside autons but our speaker center stole the show with a near 100% accurancy (we pathed into position THEN PID aligned just in case)

## Hardware
* 4x Mk4i Swerve Drive

### Rio CanBus

* 3x Falcon 500 (or more krakens)
    * 1x cancoder
* 2x Neo 1350
* 2x Neo 550
* 4x SparkMax
    * 2x E18-D80NK
* 1x CANdle

### Canivore CanBus
* 8x kraken x60
* 1x pidgeon 2
* 4x cancoder

## Software

### Driver Controls
* Left Trigger Button Shot from anywhere. auto waits until firing. (angle+Rpm+Rotation+Distance+FloorSpeed)
* Left Bumper Btn Amp  Auto Align and Shoot
* Right Bumper Btn Floor Pickup - From Floor to ready to shoot in 1 click (see Amp Automation features).
* Right Trigger Btn "Hot-N-Ready" aka baby-bird Pickup from source.
* Back btn - Trap Mode Button Sets entire control into trap mode (for longer explain See Trap Features)
* Dpad Up-right-left-down (In trap mode) Control Trap Stages
* A,B (trap Mode) Will Under/Over Index A note in the head incase of jamming)
* X,Y Xbox buttons Run Pickup head in and out manually (to clear jamz)
* X,Y (During Hot-N-Ready mode) Will set Left Or Right Source pickup (default center)
* A Button Starts an index incase The head didnt load properly
* B Button Bypasses AutoAlign incase telemetry or limelight takes severe damage
* Xbox Joysticks do the Driving

Thats it, Thats how we drove our bot, with those buttons on 1 controller. 
### Driver Information
* Lights Flash Blue when completing any main task such as Shooting/Amp/Pickup Found/Hot-n-Ready Intaking
* Green Lights will Strobe At Source Pickup If Human Player is too slow!
* While Attempting Shoot from anywhere
    * yellow lights Means cant see April tags (might be out of date odometry)
    * Rainbow lights means Can See Tags (its going in.)
    * Red Lights means We are Too Far to Shoot but Not in the passing Zone
    * Red Lights also mean were too far and not close enough to the center to pass legally
    * Green lights means Perfect Place to pass and passing is going on now.
* At 35 Seconds remaining in match lights will Turn into fire to signal End game (2 ish cycles left)
* On Bootup (9 layers of streamers and a layer of sparkle run as default)
* the spark layer is layer 0 and any other light command clears it until next default boot
* 9 blue streamers keep running throughout match until endgame fire.

### Features
* CTRE Drivetrain (Swerve with path planner)
    * Field orientated drive
    * auto orientation of FOC control Via limelight
    * Auto updating Odometry/telemetry Via limelight
    * Mega Tag Only updates if conditions are good enough for reliable data.
* Shoot from anywhere
    * Will auto adjust angle of shot and rpm to make shots from any location
    * Auto compensates for left biased shooting pattern
    * Auto Aims at specific targets but does not require april tags to line up
    * will not attempt a shot until 
        * Angle is correct (With tolerance and SettleTimer) 
        * Rpm is correct for both shooter rollers (5% tolerance with SettleTimer)
        * Rotation is within tolerance
        * Distance is between our wing and the subwoofer (our shot just couldnt half field)
        * Bots current chassis speed is below .5m/s (BUT gets Angle+Rpm ready so when we stop it pop shots instantly)
* Passing
    * Wont fire unless in center field (but Angle+RPm will ready for instant pop shots when odometry says  center field)
    * Passing target/rotation/rpm all automatically controlled for consistent results.
* Amp Automation features
    * Pizza Joe Will Auto Align at amp Via Pid Controls and using Live Updated Odometry
    * Will Auto Extend and prepare for final motion while Pid Aligning
    * Once its within alignment tolerance it will Tilt And Fire 
    * Flashes Confirmation lights to Inform driver to Get movin
    * Then quickly Resets to Go under stage ASAP 
* 1Button Floor Pickup to Fully ready for 1 button auto Fire
    * Will UnFold Out pickupHead 
        * only if note isnt already in system
        * Uses Neo 1350 Built in encoder and #25 chain to control. Never lost a step.
        * Current limited to 40 amps (in sparkmax) to protect parts
    * Will then begin intaking a note until Sparkmax Endstop is triggered (or cancelled by driver)
    * When Triggered it will continue to intake .1 seconds to give note a proper squish
    * will Fold back into passing/park position
    * when park position command is finished it will begind a delivery index
    * it will then feed the delivery holder
        * the Indexer will detect if nothing arrives and reAttempt failed feeds (no need for upper roller)
    * Indexer completes shot ready. 
* "Hot-N-Ready" aka baby-bird Pickup from source
    * Will Auto Align to Source Via Telemetry Not April Tags
    * Head Will tilt to correct pickup angle
    * Front Roller AND delivery intake rollers will intake,roll and wait for note to pass The limit Switch
    * After detection Note will then Index and is ready for shot or pass
    * Select Any pickup position (left right or center) By Pressing X Or Y button (left or right)
* Trap mode Control Features
    * Dpad UP (Before trap mode)
        * Turns on Limelight Searchlight for best telemetry
        * Aligns Pizza joe with the closest (x,y coords) Trap door (Also can use april tag)
    * Dpad Up (Trap mode)
        * Turns off limelight for viewer comfort
        * Sets Read pickup to verticle position 
        * Extend Delivery Head to edge of stage
        * The Note will Index Much further into the shooter head to Dodge the Lifting Hooks arrival
        * Reveals and extends lift chain to upper position
        * will then Tilt head to bump Lifting Hooks into position over the chain
    * Dpad Right
        * Will pull chain lift down
        * when Lifting hooks leave upper position - Re-Index the note so that it can be fired 
        * Will Flip down pickup tail momentarily to shift weight and straighten Lifting angle.
    * Dpad Left
        * Spools Shooter Head Rpm
        * Tilts head into trap
        * Tilts down Read pickup tail to change center of gravity
        * Shoots trap on week 1.
    * Dpad Down 
        * used during practice and testing to reset after shooting trap.
* Indexing System
    * runs Rollers until Limit switch triggered (uncontrolled - Crazy outside forces from pickup)
    * then will unroll until not detected
    * then will roll back in until triggered again (controlled - has no pickup forces)
    * Small amount of post roll for perfect position
    * Will detect no note and attempt to unroll and roll to get jam cleared (works during autonomous aswell)
    * Can index from back or from front and acheive perfect outcome
  
---------------------------
